FROM ros:foxy-ros-base-focal

ENV ROS_WS /ros2_ws
ENV AMENT_PREFIX_PATH /opt/ros/foxy
ENV CMAKE_PREFIX_PATH /opt/ros/foxy
ENV ROS2 /opt/ros/foxy
ENV FOXGLOVE /opt/foxglove

WORKDIR $ROS_WS

RUN apt update && apt install -y \
    ros-foxy-cv-bridge \
    python3-pip \
    python3-apt \
    python3.8-dev \
    python3-colcon-common-extensions \
    python3-vcstool \
    git \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN pip install --upgrade pip

RUN pip install carla==0.9.14 pydantic opencv-python

COPY ./src/carla_ros_interfaces /ros2_ws/src/carla_ros_interfaces
COPY ./src/custom_carla_bridge /ros2_ws/src/custom_carla_bridge

RUN . /opt/ros/foxy/setup.sh && \
    colcon build --packages-select carla_ros_interfaces custom_carla_bridge && \
    rm -rf /ros2_ws/build /ros2_ws/log

# Install carla bridge
COPY ./docker/custom_carla_bridge/entrypoint.sh /entrypoint-carla-bridge.sh
RUN chmod +x /entrypoint-carla-bridge.sh
ENTRYPOINT ["/entrypoint-carla-bridge.sh"]
