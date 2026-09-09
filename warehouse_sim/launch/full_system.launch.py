import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg_warehouse_sim = get_package_share_directory('warehouse_sim')
    pkg_warehouse_control = get_package_share_directory('warehouse_control')
    pkg_warehouse_perception = get_package_share_directory('warehouse_perception')

    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_warehouse_sim, 'launch', 'sim.launch.py')
        ),
    )

    control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_warehouse_control, 'launch', 'warehouse_control.launch.py')
        ),
    )

    perception = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_warehouse_perception, 'launch', 'perception.launch.py')
        ),
    )

    order_manager_node = Node(
        package='warehouse_order',
        executable='order_manager',
        name='order_manager',
        output='screen',
    )

    return LaunchDescription([
        sim,
        control,
        perception,
        order_manager_node,
    ])
