# ROS 2 Jazzy + rosbridge + turtlesim (offscreen) + a synthetic camera. Build context: the repo root.
FROM ros:jazzy-ros-base
RUN apt-get update && apt-get install -y --no-install-recommends \
      ros-jazzy-rosbridge-server ros-jazzy-rosapi ros-jazzy-turtlesim \
    && rm -rf /var/lib/apt/lists/*
COPY imaging.py /opt/hermes-ros/imaging.py
COPY docker/fake_camera.py docker/run_demo.sh /opt/hermes-ros/docker/
ENV QT_QPA_PLATFORM=offscreen
EXPOSE 9090
CMD ["/opt/hermes-ros/docker/run_demo.sh"]
