#!/usr/bin/env python3
"""Publishes a MarkerArray in RViz for the static box geometry in the world SDF."""

import math
import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray

IDENTITY_POSE = (0.0, 0.0, 0.0, 0.0, 0.0, 0.0)
DEFAULT_COLOR = (0.6, 0.6, 0.6, 1.0)


def parse_pose(elem):
    text = elem.findtext('pose', default=' '.join(str(v) for v in IDENTITY_POSE))
    values = [float(v) for v in text.split()]
    return tuple(values[:3]), tuple(values[3:6])


def quat_from_rpy(roll, pitch, yaw):
    cr, sr = math.cos(roll * 0.5), math.sin(roll * 0.5)
    cp, sp = math.cos(pitch * 0.5), math.sin(pitch * 0.5)
    cy, sy = math.cos(yaw * 0.5), math.sin(yaw * 0.5)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * cp * sy,
    )


def quat_mult(q1, q2):
    x1, y1, z1, w1 = q1
    x2, y2, z2, w2 = q2
    return (
        w1 * x2 + x1 * w2 + y1 * z2 - z1 * y2,
        w1 * y2 - x1 * z2 + y1 * w2 + z1 * x2,
        w1 * z2 + x1 * y2 - y1 * x2 + z1 * w2,
        w1 * w2 - x1 * x2 - y1 * y2 - z1 * z2,
    )


def quat_rotate(q, v):
    x, y, z, w = q
    vx, vy, vz = v
    # v' = q * v * q^-1, expanded for a unit quaternion.
    uvx = 2 * (y * vz - z * vy)
    uvy = 2 * (z * vx - x * vz)
    uvz = 2 * (x * vy - y * vx)
    return (
        vx + w * uvx + (y * uvz - z * uvy),
        vy + w * uvy + (z * uvx - x * uvz),
        vz + w * uvz + (x * uvy - y * uvx),
    )


def to_quat_pose(pos, rpy):
    return pos, quat_from_rpy(*rpy)


def compose(parent, child):
    """Combine two (position, quaternion) poses, parent-then-child."""
    p_pos, p_quat = parent
    c_pos, c_quat = child
    q_total = quat_mult(p_quat, c_quat)
    rotated_child_pos = quat_rotate(p_quat, c_pos)
    pos_total = tuple(p + c for p, c in zip(p_pos, rotated_child_pos))
    return pos_total, q_total


def parse_color(visual):
    ambient = visual.findtext('material/ambient')
    if ambient is None:
        return DEFAULT_COLOR
    values = [float(v) for v in ambient.split()]
    return tuple(values[:4]) if len(values) >= 4 else DEFAULT_COLOR


def parse_boxes(sdf_path):
    root = ET.parse(sdf_path).getroot()
    boxes = []
    for model in root.iter('model'):
        model_name = model.get('name', '')
        model_pose = to_quat_pose(*parse_pose(model))
        for link in model.findall('link'):
            link_name = link.get('name', '')
            link_pose = to_quat_pose(*parse_pose(link))
            model_to_link = compose(model_pose, link_pose)
            for visual in link.findall('visual'):
                size_text = visual.findtext('geometry/box/size')
                if size_text is None:
                    continue
                size = tuple(float(v) for v in size_text.split())
                visual_pose = to_quat_pose(*parse_pose(visual))
                world_pos, world_quat = compose(model_to_link, visual_pose)
                name = f"{model_name}/{link_name}/{visual.get('name', '')}"
                color = parse_color(visual)
                boxes.append((name, world_pos, world_quat, size, color))
    return boxes


class ObstacleMarkerPublisher(Node):

    def __init__(self):
        super().__init__('obstacle_marker_publisher')
        self.declare_parameter('world_path', '')
        self.declare_parameter('frame_id', 'odom')
        self.declare_parameter('publish_period_sec', 1.0)

        world_path = self.get_parameter('world_path').value
        self.frame_id = self.get_parameter('frame_id').value
        self.boxes = parse_boxes(world_path)
        self.get_logger().info(f'Loaded {len(self.boxes)} box markers from {world_path}')

        self.publisher = self.create_publisher(MarkerArray, 'obstacle_markers', 1)
        period = self.get_parameter('publish_period_sec').value
        self.timer = self.create_timer(period, self.publish_markers)

    def publish_markers(self):
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()
        for i, (name, pos, quat, size, color) in enumerate(self.boxes):
            marker = Marker()
            marker.header.frame_id = self.frame_id
            marker.header.stamp = now
            marker.ns = 'obstacles'
            marker.id = i
            marker.type = Marker.CUBE
            marker.action = Marker.ADD
            marker.pose.position.x = pos[0]
            marker.pose.position.y = pos[1]
            marker.pose.position.z = pos[2]
            marker.pose.orientation.x = quat[0]
            marker.pose.orientation.y = quat[1]
            marker.pose.orientation.z = quat[2]
            marker.pose.orientation.w = quat[3]
            marker.scale.x = size[0]
            marker.scale.y = size[1]
            marker.scale.z = size[2]
            marker.color.r = color[0]
            marker.color.g = color[1]
            marker.color.b = color[2]
            marker.color.a = color[3]
            marker.text = name
            marker_array.markers.append(marker)
        self.publisher.publish(marker_array)


def main():
    rclpy.init()
    rclpy.spin(ObstacleMarkerPublisher())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
