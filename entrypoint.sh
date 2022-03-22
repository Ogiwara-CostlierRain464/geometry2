#!/bin/bash
set -e

# setup ros2 environment
source /opt/ros/melodic/setup.bash

exec "$@"
# /bin/bash "$@"