#!/bin/bash
set -e

source /opt/ros/foxy/setup.bash
source /app/src/install/setup.bash

exec "$@"
