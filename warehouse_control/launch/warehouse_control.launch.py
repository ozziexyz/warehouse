from launch import LaunchDescription
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnProcessStart
from launch_ros.actions import Node
import math

def generate_launch_description():
    state_manager_node = Node(
        package='warehouse_control',
        executable='state_manager',
        name='state_manager',
        output='screen',
    )

    pure_pursuit_controller_node = Node(
        package='warehouse_control',
        executable='pure_pursuit_controller',
        name='pure_pursuit_controller',
        output='screen',
        parameters=[{
            'lookahead_distance': 0.2,
            'max_linear_velocity': 0.5,
            'goal_tolerance': 0.01,
            'loop_rate': 10.0,
            'turn_in_place_w': 1.5,
            'max_turn': 30 * math.pi / 180,
            'use_ground_truth': True
        }],
    )

    navigation_manager_node = Node(
        package='warehouse_control',
        executable='navigation_manager',
        name='navigation_manager',
        output='screen',
    )

    path_planner_node = Node(
        package='warehouse_control',
        executable='path_planner',
        name='path_planner',
        output='screen'
    )

    # pure_pursuit_controller starts once state_manager is running
    start_pure_pursuit_after_state_manager = RegisterEventHandler(
        OnProcessStart(
            target_action=state_manager_node,
            on_start=[pure_pursuit_controller_node],
        )
    )

    # navigation_manager depends on both the robot state service and the
    # follow_path action server, so it waits until pure_pursuit_controller
    # (which only starts after state_manager) is running
    start_navigation_manager_after_pure_pursuit = RegisterEventHandler(
        OnProcessStart(
            target_action=pure_pursuit_controller_node,
            on_start=[navigation_manager_node],
        )
    )

    return LaunchDescription([
        state_manager_node,
        path_planner_node,
        start_pure_pursuit_after_state_manager,
        start_navigation_manager_after_pure_pursuit,
    ])
