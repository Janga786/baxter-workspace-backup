#!/bin/bash
cd ~/baxter_bridge_project

# Allow X11 forwarding for GUI apps
xhost +local:docker

# Start all containers
docker compose up -d

echo "Waiting for containers to initialize..."
sleep 5

echo "Baxter Bridge System Started!"
echo ""
echo "To access containers:"
echo "  ROS1:   docker exec -it baxter_ros1_container bash"
echo "  ROS2:   docker exec -it moveit_ros2_container bash"
echo "  Bridge: docker logs -f ros_bridge_container"

