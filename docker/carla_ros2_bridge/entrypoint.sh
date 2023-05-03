#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /carla-ros-bridge/install/setup.bash

exec "$@"
