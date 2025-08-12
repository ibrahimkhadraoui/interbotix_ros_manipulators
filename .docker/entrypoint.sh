#!/bin/bash
set -e

# Source ROS 2 setup
source /opt/ros/humble/setup.bash
# Source workspace setup if it exists
if [ -f /root/interbotix_ws/install/setup.bash ]; then
    source /root/interbotix_ws/install/setup.bash
fi

exec "$@"
