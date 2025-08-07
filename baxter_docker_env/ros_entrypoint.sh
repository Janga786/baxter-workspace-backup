#!/bin/bash
set -e

# Source the main ROS setup file
source "/opt/ros/indigo/setup.bash"

# Source your compiled Baxter workspace setup file
source "/root/baxter_ws/devel/setup.bash"

# Execute the command passed to the container (e.g., "bash")
exec "$@"
