#!/bin/bash
set -e

# setup ros2 environment
source /opt/ros/$ROS_DISTRO/setup.bash
# setup slam environment
source $SLAM_WS/install/setup.bash 
source $WORKDIR/install/setup.bash

# Follow interactively
exec "$@"