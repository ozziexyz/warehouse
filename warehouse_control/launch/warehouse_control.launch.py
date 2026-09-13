import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node

def generate_launch_description():
    warehouse_layout_path = os.path.join(
        get_package_share_directory('warehouse_sim'), 'config', 'warehouse_layout.yaml'
    )

    sm_controller_node = Node(
        package='warehouse_control',
        executable='sm_controller',
        name='sm_controller',
        output='screen',
        parameters=[{
            'heading_kp': 2.0,
            'heading_kt': 0.5,
            'slowdown_distance': 0.5,
            'goal_tolerance': 0.02,
            'no_turn_distance': 0.5,
            'max_drive_angle': 0.5,
            'min_turn_angle': 0.02,
            'max_v': 0.75,
            'max_w': 1.57,
            'use_sim_time': True,
        }],
    )

    navigation_manager_node = Node(
        package='warehouse_control',
        executable='navigation_manager',
        name='navigation_manager',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    path_planner_node = Node(
        package='warehouse_control',
        executable='path_planner',
        name='path_planner',
        output='screen',
        parameters=[warehouse_layout_path, {'use_sim_time': True}],
    )

    # navigation_manager depends on the follow_path action server, so it
    # waits until sm_controller is running
    start_navigation_manager_after_sm = RegisterEventHandler(
        OnProcessStart(
            target_action=sm_controller_node,
            on_start=[navigation_manager_node],
        )
    )

    return LaunchDescription([
        path_planner_node,
        sm_controller_node,
        start_navigation_manager_after_sm,
    ])
