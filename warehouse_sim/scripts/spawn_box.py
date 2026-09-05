#!/usr/bin/env python3

import subprocess

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SpawnBox(Node):
    def __init__(self):
        super().__init__('spawn_box')

        # --- parameters, override at launch time or via CLI ---
        self.declare_parameter('sdf_path', '/path/to/model.sdf')
        self.declare_parameter('world_name', 'warehouse')
        self.declare_parameter('entity_prefix', 'spawned_box')
        self.declare_parameter('topic', 'spawn_box')

        self.sdf_path = self.get_parameter('sdf_path').value
        self.world_name = self.get_parameter('world_name').value
        self.entity_prefix = self.get_parameter('entity_prefix').value
        topic = self.get_parameter('topic').value

        self._spawn_count = 0
        self.locations = [
            (-1, 4),
            (0, 4),
            (1, 4),
            (-1, 2),
            (0, 2),
            (1, 2),
            (-1, 0),
            (0, 0),
            (1, 0),
            (-1, -2),
            (0, -2),
            (1, -2),
            (-1, -4),
            (0, -4),
            (1, -4),
        ]

        self.sub = self.create_subscription(
            Int32,
            topic,
            self.on_msg,
            10,
        )

        self.get_logger().info(
            f"Listening on '{topic}' (Int32). Will spawn '{self.sdf_path}' "
            f"into world '{self.world_name}' on every message."
        )

    def on_msg(self, msg: Int32):
        self.get_logger().info(f"Received {msg.data} -> spawning object")
        self.spawn_object(msg.data)

    def spawn_object(self, location):
        entity_name = f"{self.entity_prefix}_{self._spawn_count}"
        self._spawn_count += 1

        cmd = [
            'ros2', 'run', 'ros_gz_sim', 'create',
            '-world', self.world_name,
            '-file', self.sdf_path,
            '-name', entity_name,
            '-x', str(self.locations[location][0]),
            '-y', str(self.locations[location][1]),
            '-z', '6',
        ]

        try:
            subprocess.Popen(cmd)
            self.get_logger().info(f"Spawn requested: {entity_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to spawn object: {e}")


def main(args=None):
    rclpy.init(args=args)
    node = SpawnBox()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()