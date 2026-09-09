import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_path = os.path.join(pkg_warehouse_sim, 'worlds', 'warehouse.sdf')
    xacro_path = os.path.join(pkg_warehouse_sim, 'urdf', 'warehouse_robot.urdf.xacro')
    controller_config_path = os.path.join(pkg_warehouse_sim, 'config', 'diff_drive_controller.yaml')
    rviz_config_path = os.path.join(pkg_warehouse_sim, 'rviz', 'warehouse_sim.rviz')
    models_path = os.path.join(pkg_warehouse_sim, 'models')
    box_sdf_path = os.path.join(models_path, 'brown_box', 'model.sdf')

    # So `model://...` URIs in the world file (e.g. the floor AprilTag models) resolve
    gz_resource_path = os.pathsep.join(filter(None, [models_path, os.environ.get('GZ_SIM_RESOURCE_PATH', '')]))
    set_gz_resource_path = SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', gz_resource_path)

    robot_description_content = xacro.process_file(
        xacro_path,
        mappings={'controller_config_path': controller_config_path},
    ).toxml()

    use_rviz_arg = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Launch RViz alongside the simulation',
    )

    headless_arg = DeclareLaunchArgument(
        'headless',
        default_value='false',
        description='Run Gazebo without the GUI client',
    )

    gz_args = PythonExpression([
        "'-s -r ' if '", LaunchConfiguration('headless'), "' == 'true' else '-r '"
    ])

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': [gz_args, world_path]}.items(),
    )

    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen',
    )

    pose_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/model/warehouse_robot/pose@geometry_msgs/msg/PoseStamped[gz.msgs.Pose'],
        remappings=[
            ('/model/warehouse_robot/pose', '/ground_truth/pose'),
        ],
        output='screen',
    )

    rear_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/rear_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[
            ('/rear_camera', '/rear_camera/image_raw'),
        ],
        output='screen',
    )

    bottom_camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/bottom_camera@sensor_msgs/msg/Image@gz.msgs.Image',
        ],
        remappings=[
            ('/bottom_camera', '/bottom_camera/image_raw'),
        ],
        output='screen',
    )

    front_lidar_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/front_lidar@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan',
        ],
        remappings=[
            ('/front_lidar', '/front_lidar/scan'),
        ],
        output='screen',
    )

    imu_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/imu@sensor_msgs/msg/Imu@gz.msgs.IMU',
        ],
        output='screen',
    )

    ejector_belt_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/model/warehouse_robot/link/cage_floor_link/track_cmd_vel@std_msgs/msg/Float64]gz.msgs.Double',
        ],
        remappings=[
            ('/model/warehouse_robot/link/cage_floor_link/track_cmd_vel', '/cage_ejector/cmd_vel'),
        ],
        output='screen',
    )

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description_content,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-name', 'warehouse_robot', '-z', '0.1'],
        output='screen',
    )

    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster'],
        output='screen',
    )

    diff_drive_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['diff_drive_controller'],
        output='screen',
    )

    flap_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['flap_controller'],
        output='screen',
    )

    obstacle_marker_publisher = Node(
        package='warehouse_sim',
        executable='obstacle_marker_publisher.py',
        parameters=[{
            'world_path': world_path,
            'frame_id': 'map',
            'use_sim_time': True,
        }],
        output='screen',
    )

    spawn_box = Node(
        package='warehouse_sim',
        executable='spawn_box.py',
        parameters=[{
            'sdf_path': box_sdf_path,
            'world_name': 'warehouse',
            'use_sim_time': True,
        }],
        output='screen',
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config_path],
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_rviz')),
        output='screen',
    )

    gt_map_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base_to_lidar',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'warehouse',
            '--child-frame-id', 'map'
        ]
    )

    pose_to_tf = Node(
        package='warehouse_sim',
        executable='pose_to_tf.py'
    )

    lidar_robot_pub = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_pub_base_to_lidar',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'front_lidar_link',
            '--child-frame-id', 'warehouse_robot/base_footprint/front_lidar'
        ]
    )

    # gt_base_pub = Node(
    #         package='tf2_ros',
    #         executable='static_transform_publisher',
    #         name='static_tf_pub_base_to_lidar',
    #         arguments=[
    #             '--x', '0.0',
    #             '--y', '0.0',
    #             '--z', '0.0',
    #             '--roll', '0.0',
    #             '--pitch', '0.0',
    #             '--yaw', '0.0',
    #             '--frame-id', 'ground_truth',
    #             '--child-frame-id', 'base_footprint'
    #         ]
    #     )

    delayed_joint_state_broadcaster_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[joint_state_broadcaster_spawner],
        )
    )

    delayed_diff_drive_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=joint_state_broadcaster_spawner,
            on_exit=[diff_drive_controller_spawner],
        )
    )

    delayed_flap_controller_spawner = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=diff_drive_controller_spawner,
            on_exit=[flap_controller_spawner],
        )
    )

    return LaunchDescription([
        use_rviz_arg,
        headless_arg,
        set_gz_resource_path,
        gz_sim,
        clock_bridge,
        gt_map_pub,
        pose_to_tf,
        pose_bridge,
        rear_camera_bridge,
        bottom_camera_bridge,
        front_lidar_bridge,
        # gt_base_pub,
        lidar_robot_pub,
        imu_bridge,
        ejector_belt_bridge,
        robot_state_publisher,
        spawn_robot,
        delayed_joint_state_broadcaster_spawner,
        delayed_diff_drive_controller_spawner,
        delayed_flap_controller_spawner,
        obstacle_marker_publisher,
        spawn_box,
        rviz,
    ])
