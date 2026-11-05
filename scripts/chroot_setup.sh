#!/bin/bash
# Runs INSIDE an Ubuntu 24.04 chroot: installs ROS 2 Jazzy + rosbridge + turtlesim + python tooling. (Blaxel has no docker.)
set -euo pipefail
export DEBIAN_FRONTEND=noninteractive
apt-get update -qq
apt-get install -y -qq --no-install-recommends ca-certificates curl gnupg git python3 python3-venv python3-pip >/dev/null
curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=amd64 signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu noble main" > /etc/apt/sources.list.d/ros2.list
apt-get update -qq
apt-get install -y -qq --no-install-recommends ros-jazzy-ros-core ros-jazzy-rosbridge-server ros-jazzy-rosapi \
  ros-jazzy-turtlesim ros-jazzy-sensor-msgs ros-jazzy-geometry-msgs >/dev/null
apt-get clean; rm -rf /var/lib/apt/lists/*
echo "chroot setup done"; df -h / | tail -1
