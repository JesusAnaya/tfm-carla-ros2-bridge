# Stage 1: Build ROS2 workspace
FROM  ros:humble-ros-base AS builder

WORKDIR /app

COPY ./src/carla_ros_interfaces /app/src/carla_ros_interfaces

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select carla_ros_interfaces && \
    rm -rf /app/build /app/log

# Stage 2: Build ROS2 workspace
FROM  ros:humble-ros-core AS base
RUN apt update && apt install -y \
    ros-humble-foxglove-bridge \
    && rm -rf /var/lib/apt/lists/*

COPY --from=builder /app/install /app/install

COPY ./docker/foxglove_bridge/entrypoint.sh /entrypoint-foxglove-bridge.sh
RUN chmod +x /entrypoint-foxglove-bridge.sh
ENTRYPOINT ["/entrypoint-foxglove-bridge.sh"]
