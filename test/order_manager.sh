#!/bin/bash
ros2 topic pub --once /order std_msgs/msg/Int32MultiArray "{data: [$1, $2, $3]}"