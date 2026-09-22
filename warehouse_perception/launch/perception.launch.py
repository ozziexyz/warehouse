import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_warehouse_perception = get_package_share_directory('warehouse_perception')
    tags_config_path = os.path.join(pkg_warehouse_perception, 'config', 'tags_36h11.yaml')
    tag_tf_params = PathJoinSubstitution([pkg_warehouse_perception, 'config', 'tag_tf.yaml'])
    local_ekf_params = os.path.join(pkg_warehouse_perception, 'config', 'local_ekf.yaml')
    global_ekf_params = os.path.join(pkg_warehouse_perception, 'config', 'global_ekf.yaml')

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='bottom_camera',
        description='Base topic namespace of the camera to run AprilTag detection on',
    )
    camera_topic = LaunchConfiguration('camera_topic')

    camera_info_publisher = Node(
        package='warehouse_perception',
        executable='static_camerainfo_publisher',
        name='static_camerainfo_publisher',
        remappings=[
            ('image_raw', [camera_topic, '/image_raw']),
            ('camera_info', [camera_topic, '/camera_info'])
        ],
        parameters=[{'use_sim_time': True}]
    )

    # apriltag_ros needs a rectified image; the simulated camera only publishes image_raw
    rectify_node = Node(
        package='image_proc',
        executable='rectify_node',
        name='rectify',
        remappings=[
            ('image', [camera_topic, '/image_raw']),
            ('camera_info', [camera_topic, '/camera_info']),
            ('image_rect', [camera_topic, '/image_rect']),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}]
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        remappings=[
            ('image_rect', [camera_topic, '/image_rect']),
            ('camera_info', [camera_topic, '/camera_info']),
        ],
        parameters=[tags_config_path, {'use_sim_time': True}],
        output='screen',
    )

    apriltag_localization = Node(
        package='warehouse_perception',
        executable='apriltag_localization',
        parameters=[tag_tf_params, {'use_sim_time': True}]
    )

    local_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='local_ekf_node',
        output='screen',
        parameters=[local_ekf_params],
        remappings=[
            ('odometry/filtered', '/odometry/local'),
        ],
    )

    global_ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='global_ekf_node',
        output='screen',
        parameters=[global_ekf_params]
    )

    return LaunchDescription([
        camera_topic_arg,
        rectify_node,
        apriltag_node,
        camera_info_publisher,
        apriltag_localization,
        local_ekf_node,
        global_ekf_node
    ])
