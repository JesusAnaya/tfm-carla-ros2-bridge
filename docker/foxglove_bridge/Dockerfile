FROM  osrf/ros:humble-desktop AS base

WORKDIR /app

RUN apt update && apt install -y ros-humble-foxglove-bridge

COPY ./src/carla_ros_interfaces /app/src/carla_ros_interfaces

RUN . /opt/ros/humble/setup.sh && \
    colcon build --packages-select carla_ros_interfaces && \
    rm -rf /app/build /app/log

COPY ./docker/foxglove_bridge/entrypoint.sh /entrypoint-foxglove-bridge.sh
RUN chmod +x /entrypoint-foxglove-bridge.sh
ENTRYPOINT ["/entrypoint-foxglove-bridge.sh"]
