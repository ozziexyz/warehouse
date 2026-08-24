import os

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler
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

    camera_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/front_camera@sensor_msgs/msg/Image@gz.msgs.Image',
            '/front_camera_info@sensor_msgs/msg/CameraInfo@gz.msgs.CameraInfo',
        ],
        remappings=[
            ('/front_camera', '/front_camera/image_raw'),
            ('/front_camera_info', '/front_camera/camera_info'),
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
            'frame_id': 'odom',
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
        gz_sim,
        clock_bridge,
        camera_bridge,
        robot_state_publisher,
        spawn_robot,
        delayed_joint_state_broadcaster_spawner,
        delayed_diff_drive_controller_spawner,
        delayed_flap_controller_spawner,
        obstacle_marker_publisher,
        rviz,
    ])
