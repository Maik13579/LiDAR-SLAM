#https://docs.clearpathrobotics.com/docs/ros/tutorials/simulator/install/
FROM slam
ENV WORKDIR /root/clearpath_ws
RUN apt update && apt install -y \
    wget \
    && rm -rf /var/lib/apt/lists/*

RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list'
RUN wget http://packages.osrfoundation.org/gazebo.key -O - | apt-key add -
RUN apt update && apt install -y \
    ignition-fortress \
    ros-humble-clearpath-desktop \
    ros-$ROS_DISTRO-teleop-twist-joy \
    python3-vcstool \
&& rm -rf /var/lib/apt/lists/*

COPY clearpath_simulator $WORKDIR/src/clearpath_simulator

WORKDIR $WORKDIR
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
&& apt update && rosdep update \
&& rosdep install -r --from-paths src -i -y \
&& rm -rf /var/lib/apt/lists/*

WORKDIR $WORKDIR
RUN source /opt/ros/$ROS_DISTRO/setup.bash \
&& colcon build --symlink-install \
&& echo "source $WORKDIR/install/setup.bash" >> ~/.bashrc

COPY entrypoint.bash /entrypoint.bash
RUN chmod +x /entrypoint.bash

ENTRYPOINT ["/entrypoint.bash"]
COPY world /world
COPY world/myworld.sdf /root/clearpath_ws/install/clearpath_gz/share/clearpath_gz/worlds/myworld.sdf