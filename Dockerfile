FROM ros:humble-ros-base

RUN apt-get update && apt-get install -y --no-install-recommends \
      libglfw3-dev libglew-dev libgl1-mesa-dev \
      ros-humble-tf2-geometry-msgs ros-humble-visualization-msgs \
      ros-humble-ament-cmake-gtest \
    && rm -rf /var/lib/apt/lists/*

WORKDIR /ws/src/astra_slam
COPY . .
WORKDIR /ws
RUN . /opt/ros/humble/setup.sh && colcon build --symlink-install

CMD ["bash", "-lc", "source /opt/ros/humble/setup.bash && source /ws/install/setup.bash && ros2 launch astra_slam demo.launch.py"]
