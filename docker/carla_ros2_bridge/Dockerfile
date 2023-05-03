# Use ROS 2 Foxy base image
FROM ros:foxy-ros-base-focal

# Set up environment
ENV ROS_WS /carla-ros-bridge

# Install dependencies
RUN apt-get update && \
    apt-get install -y \
        git \
        software-properties-common \
        && rm -rf /var/lib/apt/lists/*

# Install pip for Python
RUN apt-get update && \
    apt-get install -y python3-pip && \
    pip install --upgrade pip

# Clone the CARLA ROS Bridge repository
RUN mkdir -p ${ROS_WS}/src && \
    cd ${ROS_WS}/src && \
    git clone --recurse-submodules https://github.com/carla-simulator/ros-bridge.git

# Install ROS dependencies
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    cd ${ROS_WS} && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y"

# Build the ROS bridge workspace
RUN /bin/bash -c "source /opt/ros/foxy/setup.bash && \
    cd ${ROS_WS} && \
    colcon build"

# Install CARLA Python API
RUN pip3 install carla==0.9.13 nose2 setuptools pygame

# Set up the entrypoint
COPY ./entrypoint.sh /
RUN chmod +x /entrypoint.sh
ENTRYPOINT ["/entrypoint.sh"]
