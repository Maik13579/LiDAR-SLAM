This is a fork from [Kitware](https://gitlab.kitware.com/keu-computervision/slam/-/tree/856af04ca3fd758b8f6fb2fc168d17aa63a3481d)

![SLAM in LidarView](doc/paraview_plugin.png)

![SLAM in ROS](doc/ros_node.png)

# LiDAR SLAM

- [LiDAR SLAM](#lidar-slam)
  - [Introduction and contents](#introduction-and-contents)
  - [Core SLAM lib dependencies](#core-slam-lib-dependencies)
  - [ROS2 Dependencies](#ros2-dependencies)
  - [Installation](#installation)
  - [Testing Sim](#testing-sim)


## Introduction and contents

This repo contains LiDAR-only visual SLAM as a ROS2 wrapper for easier use.

It has been successfully tested on data from several common LiDAR sensors:
- Velodyne (VLP-16, VLP-32c, HDL-32, HDL-64, VLS-128)
- Ouster (OS0/1/2-32/64/128)
- RoboSense (RS-LiDAR-16 RS-LiDAR-32)
- Hesai (PandarXT16, PandarXT32, Pandar128)

Have a look at kitware's [SLAM demo video](https://vimeo.com/524848891)!

Repo contents :
- `slam_lib/` : core *LidarSlam* library containing SLAM algorithm and other utilities.
- `ros2_wrapping/`: ROS2 packages to enable SLAM use on a ROS2 system.
- `CMakeLists.txt` : *CMakeLists* used to call to build core *LidarSlam* lib and *paraview_wrapping*.
- `testing_sim/` : A Gazebo Turtlebot3 (with velodyne) simulation used for testing.


See [ros2_wrapping/lidar_slam/README.md](ros2_wrapping/lidar_slam/README.md) for more details.

## Core SLAM lib dependencies

Dependencies are listed in the table below along with the version used during development and testing. Minimum required versions have not been determined yet.

| Dependency | Minimum tested Version |
| :--------: | :--------------------: |
| Eigen3     | 3.3.4                  |
| Ceres      | 1.13.0                 |
| PCL        | 1.8                    |
| nanoflann  | 1.3.0                  |
| g2o*       | 1.0.0 (master)         |
| OpenMP*    | 2.0                    |
| gtsam*     | 4.2a8                  |
| OpenCV*    | 4.5.4                  |

(*) optional dependencies :

- If G2O is not available (or disabled), *LidarSlam* lib will still be compiled, but without pose graph optimization features.
- If GTSAM is not available (or disabled), *LidarSlam* lib will still be compiled, but without IMU processing features.
- If OpenCV is not available (or disabled), *LidarSlam* lib will still be compiled, but without camera features.
- If OpenMP is available, it is possible to use multi-threading to run some SLAM steps in parallel and achieve higher processing speed.
- If OpenCV is not available (or disabled), *LidarSlam* lib will still be compiled, but without camera integration.

**/!\ Warning** Make sure to compile/install G2O with the same Ceres version as the one used in the SLAM compilation. To do so, disable the feature [G2O_USE_VENDORED_CERES](https://github.com/RainerKuemmerle/g2o/blob/master/CMakeLists.txt) during G2O compilation and link against the right version of Ceres.

## ROS2 Dependencies

Ensure all *LidarSlam* dependencies are respected (see next sections to do so). Specific ROS packages dependencies are listed in the table below along with the version used during development and testing.

| Dependency      | Tested Versions | Install (`sudo apt-get install <pkg>`)                                             | status    |
|:---------------:|:---------------:|:----------------------------------------------------------------------------------:|:---------:|
| ROS             | humble/iron     | `ros-$ROS_DISTRO-desktop-full`                                                     | mandatory |
| pcl-ros         | 1.7.4           | `ros-$ROS_DISTRO-pcl-ros`                                                          | mandatory |
| gps_common      | 0.3.0           | `ros-$ROS_DISTRO-gps-common`                                                       | optional  |
| apriltag        | 3.2.0           | `ros-$ROS_DISTRO-apriltag`                                                         | optional  |
| g2o             | 5.3             | `ros-$ROS_DISTRO-libg2o`                                                           | optional  |


For Velodyne usage, please note that the ROS Velodyne driver with minimum version 1.6 is needed.
Be careful, this ROS Velodyne driver 1.6 is not backward-compatible with previous versions.
You can install the new Velodyne driver using the command `sudo apt install ros-$ROS_DISTRO-velodyne`.

For Ouster usage, the driver can be found in this [git repo](https://github.com/ouster-lidar/ouster-ros/tree/ros2). We recommand to build it on another workspace but you can also clone it under the ros2_wrapping folder and build it at the same time as the SLAM.


## Installation

You can build a ROS2 Humble docker image with :

```bash
docker compose -f docker/compose.yaml build
```

And start it with :

```bash
docker compose -f docker/compose.yaml up
```


## Testing Sim

There is a gazebo turtlebot 3 simulation (with velodyne) in the testing_sim folder.

After you build the slam image you can start the simulation with :

```bash
git submodule update --init
docker compose -f testing_sim/compose.yaml up
```

Starting it for the first time can take a while.