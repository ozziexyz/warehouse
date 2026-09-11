#!/bin/bash

ros2 action send_goal /navigate_to_pose warehouse_interfaces/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: $1, y: $1, z: 0.0}, orientation: {w: 1.0}}}}" \
  --feedback
