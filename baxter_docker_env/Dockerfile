# Start from the official ROS Indigo base image
FROM ros:indigo

# Install all necessary system dependencies in one go
RUN apt-get update && apt-get install -y \
    build-essential \
    cmake \
    git \
    python-rosdep \
    ros-indigo-moveit-full \
    unzip \
    && rm -rf /var/lib/apt/lists/*

# Fix for rosdep init error
RUN if [ -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then rm /etc/ros/rosdep/sources.list.d/20-default.list; fi \
    && rosdep init \
    && rosdep update

# Create and move into the catkin workspace
RUN mkdir -p /root/baxter_ws/src
WORKDIR /root/baxter_ws

# --- Use the pre-downloaded source code ---
# Copy the 'src' folder (containing our ZIP files) from the build context into the image
COPY ./src /root/baxter_ws/src

# Unzip, rename, and clean up the source code folders
RUN cd src \
    && unzip baxter-sdk.zip && mv baxter-1.2.0 baxter && rm baxter-sdk.zip \
    && unzip baxter_moveit_config.zip && mv baxter_moveit_config-indigo-devel baxter_moveit_config && rm baxter_moveit_config.zip

# --- Install Dependencies and Build ---
# Install all ROS dependencies for the packages we just added
RUN /bin/bash -c "source /opt/ros/indigo/setup.bash; rosdep install --from-paths src --ignore-src -y"

# Build the workspace with catkin_make
RUN /bin/bash -c "source /opt/ros/indigo/setup.bash; catkin_make"

# --- Set up the Entrypoint ---
# This script will be run every time the container starts,
# ensuring our environment is always ready.
COPY ./ros_entrypoint.sh /
RUN chmod +x /ros_entrypoint.sh
ENTRYPOINT ["/ros_entrypoint.sh"]
CMD ["bash"]
