FROM carla-ros2-bridge:latest

# Set the working directory
WORKDIR /app

# Copy the tfm_carla_data_collector package into the container's workspace
COPY --chown=1000:1000 ./src/tfm_carla_data_collector /app/src/tfm_carla_data_collector

# Install std_msgs Python package
RUN pip3 install -r /app/src/tfm_carla_data_collector/requirements.txt

# Build the package

RUN /bin/bash -c "cd /app/src/ && \
    source /opt/ros/foxy/setup.bash && \
    colcon build --packages-select tfm_carla_data_collector && \
    rm -r /app/src/build /app/src/log"

COPY ./docker/tfm_carla_data_collector/entrypoint.sh /

RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
