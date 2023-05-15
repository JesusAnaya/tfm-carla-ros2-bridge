FROM carlasim/carla:0.9.14

USER root

RUN apt-get update && \
    apt-get install -y xdg-user-dirs && \
    rm -rf /var/lib/apt/lists/*

USER carla

WORKDIR /home/carla

CMD ["/bin/bash"]
