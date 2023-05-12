# Simple and Customized Carla ROS2 Bridge for TFM Master's Project

This repository contains the code for my TFM Master's project, a bridge between the CARLA simulator and ROS2 using custom nodes.

## Table of Contents

- [Dependencies](#dependencies)
- [Building](#building)
- [Running](#running)
- [Customizing](#customizing)
- [References](#references)

## Dependencies

This project requires Docker and Docker Compose to run. Ensure that you have these installed on your local environment before proceeding.

## Building

The project uses Docker Compose to handle its environment. In order to build the Docker images, run the following command in the root directory of the project:

```bash
docker-compose build
```

To compile the ROS modules in the packages, run:

```bash
docker-compose run custom_carla_bridge bash -c "cd /ros2_ws && colcon build && rm -rf ./build ./log"
```

To build a single package, use the following command:

```bash
docker-compose run custom_carla_bridge bash -c "cd /ros2_ws && colcon build --packages-select custom_carla_bridge && rm -rf ./build ./log"
```

## Running

After successfully building the project, you can start the node with the following command:

```bash
docker-compose up
```

You can customize the parameters of the node by editing the `docker-compose.yml` file.

```bash
ros2 launch custom_carla_bridge carla_ros_node.launch.py host:=192.168.1.73 town:=Town07_Opt
```

## Customizing

The command above can be customized as needed for your local environment. Replace 192.168.1.73 with the IP address of your host machine and Town07_Opt with the specific town you want to simulate.

## References

For more information on the CARLA simulator and ROS2, check out the following links:

- [CARLA Simulator](https://carla.org/)
- [ROS2 Documentation](https://docs.ros.org/en/foxy/index.html)

--------------------
Please make sure to replace the placeholders with the appropriate values for your project.

License: [Apache 2.0](LICENSE)
