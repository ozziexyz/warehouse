#!/usr/bin/env python3
"""Publishes a MarkerArray in RViz for the static obstacle models in the world SDF."""

import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray


def parse_obstacles(sdf_path):
    root = ET.parse(sdf_path).getroot()
    obstacles = []
    for model in root.iter('model'):
        name = model.get('name', '')
        box = model.find('.//box/size')
        if not name.startswith('obstacle_') or box is None:
            continue
        pose = [float(v) for v in model.findtext('pose', default='0 0 0 0 0 0').split()]
        size = [float(v) for v in box.text.split()]
        obstacles.append((name, pose, size))
    return obstacles


def quaternion_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


class ObstacleMarkerPublisher(Node):

    def __init__(self):
        super().__init__('obstacle_marker_publisher')
        self.declare_parameter('world_path', '')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_period_sec', 1.0)

        world_path = self.get_parameter('world_path').value
        self.frame_id = self.get_parameter('frame_id').value
        self.obstacles = parse_obstacles(world_path)
        self.get_logger().info(f'Loaded {len(self.obstacles)} obstacles from {world_path}')

        self.publisher = self.create_publisher(MarkerArray, 'obstacle_markers', 1)
        period = self.get_parameter('publish_period_sec').value
        self.timer = self.create_timer(period, self.publish_markers)

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (name, pose, size) in enumerate(self.obstacles):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = pose[0]
            marker.pose.position.y = pose[1]
            marker.pose.position.z = pose[2]
            qx, qy, qz, qw = quaternion_from_rpy(pose[3], pose[4], pose[5])
            marker.pose.orientation.x = qx
            marker.pose.orientation.y = qy
            marker.pose.orientation.z = qz
            marker.pose.orientation.w = qw
            marker.scale.x = size[0]
            marker.scale.y = size[1]
            marker.scale.z = size[2]
            marker.color.r = 0.6
            marker.color.g = 0.6
            marker.color.b = 0.6
            marker.color.a = 1.0
            marker.text = name
            marker_array.markers.append(marker)
        self.publisher.publish(marker_array)


def main():
    rclpy.init()
    rclpy.spin(ObstacleMarkerPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
