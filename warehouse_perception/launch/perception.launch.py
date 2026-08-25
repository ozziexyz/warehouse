import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_warehouse_perception = get_package_share_directory('warehouse_perception')
    tags_config_path = os.path.join(pkg_warehouse_perception, 'config', 'tags_36h11.yaml')

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='front_camera',
        description='Base topic namespace of the camera to run AprilTag detection on',
    )
    camera_topic = LaunchConfiguration('camera_topic')

    camera_info_publisher = Node(
        package='warehouse_perception',
        executable='static_camerainfo_publisher',
        name='rear_camerainfo_publisher',
        remappings=[
            ('image_raw', [camera_topic, '/image_raw']),
            ('camera_info', [camera_topic, '/camera_info'])
        ]
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
    )

    apriltag_node = Node(
        package='apriltag_ros',
        executable='apriltag_node',
        name='apriltag',
        remappings=[
            ('image_rect', [camera_topic, '/image_rect']),
            ('camera_info', [camera_topic, '/camera_info']),
        ],
        parameters=[tags_config_path],
        output='screen',
    )

    return LaunchDescription([
        camera_topic_arg,
        rectify_node,
        apriltag_node,
        camera_info_publisher
    ])
