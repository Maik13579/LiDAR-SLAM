ARG BASE_IMAGE=ros:humble
FROM $BASE_IMAGE AS base

SHELL ["/bin/bash", "-c"]
ENV DEBIAN_FRONTEND=noninteractive
ENV RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

ENV SLAM_WS=/root/slam_ws

# Install Slam ROS dependencies
RUN apt update && apt install -y \
  ros-$ROS_DISTRO-rmw-cyclonedds-cpp \
  ros-$ROS_DISTRO-rviz2 \
  ros-$ROS_DISTRO-libg2o \
  ros-$ROS_DISTRO-velodyne \
  ros-$ROS_DISTRO-pcl-ros \
  libgl1-mesa-glx libgl1-mesa-dri xvfb \
&& rm -rf /var/lib/apt/lists/*

# Install Slam dependencies
WORKDIR /
COPY docker/deps_install_scripts/install_deps.sh install_deps.sh
RUN sh install_deps.sh
COPY docker/deps_install_scripts/install_ceres.sh install_ceres.sh
RUN sh install_ceres.sh
COPY docker/deps_install_scripts/install_nanoflann.sh install_nanoflann.sh
RUN sh install_nanoflann.sh

# Resolve dependencies with rosdep
# Copy only the package.xml files to use cache
COPY ros2_wrapping/lidar_conversions/package.xml ${SLAM_WS}/src/slam/ros2_wrapping/lidar_conversions/package.xml
COPY ros2_wrapping/lidar_slam/package.xml ${SLAM_WS}/src/slam/ros2_wrapping/lidar_slam/package.xml
COPY ros2_wrapping/slam_visualization/package.xml ${SLAM_WS}/src/slam/ros2_wrapping/slam_visualization/package.xml
RUN apt update && \
    rosdep update && \
    rosdep install --from-paths ${SLAM_WS}/src --ignore-src -r -y \
    && rm -rf /var/lib/apt/lists/*

FROM base AS build 
COPY cmake ${SLAM_WS}/src/slam/cmake
COPY slam_lib ${SLAM_WS}/src/slam/slam_lib
COPY ros2_wrapping ${SLAM_WS}/src/slam/ros2_wrapping
COPY CMakeLists.txt ${SLAM_WS}/src/slam/CMakeLists.txt
WORKDIR ${SLAM_WS}
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
    && colcon build --base-paths src/slam/ros2_wrapping --cmake-args -DCMAKE_BUILD_TYPE=Release -DENABLE_g2o=true

RUN echo "source /opt/ros/$ROS_DISTRO/setup.bash" >> ~/.bashrc
RUN echo "source ${SLAM_WS}/install/setup.bash" >> ~/.bashrc

COPY docker/slam_entrypoint.bash /slam_entrypoint.bash
RUN chmod +x /slam_entrypoint.bash
ENTRYPOINT ["/slam_entrypoint.bash"]