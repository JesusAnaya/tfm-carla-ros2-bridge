#!/bin/bash

set -e
# setup ros2 environment
source /opt/ros/foxy/setup.bash
source /ros2_ws/install/local_setup.bash

exec "$@"
