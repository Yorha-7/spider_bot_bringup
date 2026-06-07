#!/usr/bin/env bash
# Big Bertha bringup — environment install for Ubuntu 24.04
# Installs: ROS 2 Jazzy + Gazebo Harmonic + Nav2 + SLAM + ros2_control + tooling
# Safe to re-run (idempotent-ish). Requires sudo.
set -euo pipefail

echo "==> [1/6] Locale"
sudo apt update
sudo apt install -y locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

echo "==> [2/6] ROS 2 apt repository (ros2-apt-source)"
sudo apt install -y software-properties-common curl
sudo add-apt-repository -y universe
ROS_APT_SOURCE_VERSION="$(curl -s https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
CODENAME="$(. /etc/os-release && echo "$VERSION_CODENAME")"
curl -L -o /tmp/ros2-apt-source.deb \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${CODENAME}_all.deb"
sudo apt install -y /tmp/ros2-apt-source.deb

echo "==> [3/6] Update + base ROS 2 Jazzy desktop"
sudo apt update
sudo apt upgrade -y
sudo apt install -y ros-jazzy-desktop ros-dev-tools

echo "==> [4/6] Gazebo Harmonic + ros_gz + ros2_control"
sudo apt install -y \
  ros-jazzy-ros-gz \
  ros-jazzy-gz-ros2-control \
  ros-jazzy-ros2-control \
  ros-jazzy-ros2-controllers \
  ros-jazzy-controller-manager

echo "==> [5/6] Nav2 + SLAM + localization + model/teleop helpers"
sudo apt install -y \
  ros-jazzy-navigation2 \
  ros-jazzy-nav2-bringup \
  ros-jazzy-slam-toolbox \
  ros-jazzy-robot-localization \
  ros-jazzy-xacro \
  ros-jazzy-joint-state-publisher \
  ros-jazzy-joint-state-publisher-gui \
  ros-jazzy-teleop-twist-keyboard \
  ros-jazzy-tf2-tools

echo "==> [6/6] Build/test tooling + rosdep + git-lfs"
sudo apt install -y python3-colcon-common-extensions python3-rosdep git-lfs
sudo rosdep init 2>/dev/null || true
rosdep update

# Source ROS by default in new shells (no-op if already present)
if ! grep -q "source /opt/ros/jazzy/setup.bash" "$HOME/.bashrc"; then
  echo "source /opt/ros/jazzy/setup.bash" >> "$HOME/.bashrc"
fi

echo ""
echo "==================================================================="
echo " DONE. Open a NEW terminal (or: source /opt/ros/jazzy/setup.bash)"
echo " Verify:   ros2 doctor --report   &&   gz sim --version"
echo "==================================================================="
