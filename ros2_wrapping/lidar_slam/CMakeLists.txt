cmake_minimum_required(VERSION 3.9)
project(lidar_slam VERSION 2.2)

# Compile as C++14, supported in ROS Kinetic and newer
set(CMAKE_CXX_STANDARD 14)
if (NOT CMAKE_BUILD_TYPE)
    message(STATUS "ROS2 Wrapping : No build type selected, default to RelWithDebInfo")
    set(CMAKE_BUILD_TYPE RelWithDebInfo)
endif()

option(BUILD_SLAM_LIB "Build the SLAM library before the ROS2 package" ON)

# Add binobj to increase address capacity on Windows
if (WIN32)
  add_compile_options(/bigobj)
endif()

##################
## Dependencies ##
##################

# Find ament macros and libraries
find_package(ament_cmake REQUIRED)
# Generate custom interfaces
find_package(rosidl_default_generators REQUIRED)

# Dependencies of LidarSlam
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(lidar_slam_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(std_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(apriltag_ros REQUIRED)
find_package(visualization_msgs REQUIRED)

find_package(cv_bridge)

###########
## Build ##
###########

# To find shared libraries used by the project at runtime
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)

# Build LidarSlam lib which lies in parent directory
if (BUILD_SLAM_LIB)
  set(slamlib_from_subdirectory true)
  add_subdirectory(../.. ${CMAKE_BINARY_DIR}/slam)
  set(LidarSlam_target LidarSlam)
else()
  find_package(LidarSlam REQUIRED)
  set(LidarSlam_target LidarSlam::LidarSlam)
endif()

# Add LiDAR SLAM ROS node, link dependencies and librairies
add_executable(lidar_slam_node
  src/LidarSlamNode.cxx
  src/LidarSlamNode_main.cxx
)

# Optionnal dependency cv_bridge
if(cv_bridge_FOUND)
  message(STATUS "ROS2 wrapping : cv_bridge was found, camera interface is built")
  list(APPEND optional_DEP cv_bridge)
  list(APPEND optional_LIBRARIES ${cv_bridge_LIBRARIES})
  # Anything else you need to do to enable use of the optional dep, like add definitions
  target_compile_definitions(lidar_slam_node PUBLIC "-DUSE_CV_BRIDGE")
else()
  message(STATUS "ROS wrapping : cv_bridge was not found, camera interface cannot be used")
endif()

set(lidar_slam_dependencies
  rclcpp
  lidar_slam_interfaces
  tf2_ros
  pcl_conversions
  std_msgs
  nav_msgs
  geometry_msgs
  sensor_msgs
  apriltag_ros
  ${optional_DEP}
)

ament_target_dependencies(lidar_slam_node
  ${lidar_slam_dependencies}
)

target_link_libraries(lidar_slam_node
  ${LidarSlam_target}
  "${cpp_typesupport_target}"
  ${optional_LIBRARIES}
  )

# Add aggregation node, link dependencies and librairies
add_executable(aggregation_node
  src/AggregationNode.cxx
)
ament_target_dependencies(aggregation_node
  rclcpp
  lidar_slam_interfaces
  pcl_conversions
  std_msgs
  nav_msgs
  tf2_ros
  visualization_msgs
)
target_link_libraries(aggregation_node
  ${LidarSlam_target}
  "${cpp_typesupport_target}"
)

#############
## Install ##
#############

install(TARGETS lidar_slam_node aggregation_node
        DESTINATION lib/${PROJECT_NAME})
install(DIRECTORY launch params
        DESTINATION share/${PROJECT_NAME})

# Installs the package.xml
# Installs config (and possibly target) files for CMake so that it can be found by other packages
ament_export_dependencies(${lidar_slam_dependencies})
ament_package()