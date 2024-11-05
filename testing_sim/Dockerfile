#https://discourse.ros.org/t/added-velodyne-simulator-lidar-to-turtlebot3-models/38482
FROM slam
ENV ROS2_WS /root/ros2_ws
RUN apt update && apt install -y \
    ros-$ROS_DISTRO-velodyne-simulator \
    ros-$ROS_DISTRO-teleop-twist-joy \
    wget \
    && rm -rf /var/lib/apt/lists/*

COPY turtlebot3_simulations $ROS2_WS/src/turtlebot3_simulations
#overwrite model
COPY tb3.sdf $ROS2_WS/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_burger/model.sdf

# Download nicer meshes
WORKDIR $ROS2_WS/src/turtlebot3_simulations/turtlebot3_gazebo/models/turtlebot3_common/meshes
RUN rm -rf tire.dae burger_base.dae
RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3_description/meshes/bases/burger_base.stl
RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3_description/meshes/wheels/left_tire.stl
RUN wget https://raw.githubusercontent.com/ROBOTIS-GIT/turtlebot3/ros2/turtlebot3_description/meshes/wheels/right_tire.stl

# Install rosdeps
WORKDIR $ROS2_WS
RUN apt update \
    && rosdep update \
    && rosdep install -y --ignore-src --from-paths src \
    && rm -rf /var/lib/apt/lists/*

# Build
WORKDIR $ROS2_WS
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release \
    && echo "source $ROS2_WS/install/setup.bash" >> ~/.bashrc

COPY sim.launch.py /launch/sim.launch.py
COPY joystick.yaml /launch/joystick.yaml

# Setup turtlebot model
ENV TURTLEBOT3_MODEL burger

COPY entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash
ENTRYPOINT ["/entrypoint.bash"]