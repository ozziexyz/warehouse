#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, TransformStamped
from tf2_ros import TransformBroadcaster

class PoseToTF(Node):
    def __init__(self):
        super().__init__('pose_to_tf')
        self.br = TransformBroadcaster(self)
        self.child_frame = '/ground_truth' 
        self.sub = self.create_subscription(
            PoseStamped, '/ground_truth/pose', self.pose_callback, 10)

    def pose_callback(self, msg: PoseStamped):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = "map"   # parent frame
        t.child_frame_id = self.child_frame        # new dynamic frame

        t.transform.translation.x = msg.pose.position.x
        t.transform.translation.y = msg.pose.position.y
        t.transform.translation.z = msg.pose.position.z
        t.transform.rotation = msg.pose.orientation

        self.br.sendTransform(t)

def main():
    rclpy.init()
    node = PoseToTF()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()