from email.policy import default
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch_xml.launch_description_sources import XMLLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')
  slam_launch_path = os.path.join(lidar_slam_share_path, 'launch', 'slam.launch.py')
  lidar_conversion_share_path = get_package_share_directory('lidar_conversions')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("replay", default_value="true", description="Whether to process live or replayed data"),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    DeclareLaunchArgument("os_driver", default_value="false", description="If true, activate os_node."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    DeclareLaunchArgument("camera_topic", default_value="camera", description="topic from which to get the rgb camera data"),
    DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="topic from which to get the rgb camera info"),
    DeclareLaunchArgument("driver_parameter_file", default_value=os.path.join(lidar_slam_share_path, 'params', "ouster_driver_parameters.yaml"),
                          description="Path to the file containing Ouster driver parameters"),
    DeclareLaunchArgument("metadata_in", default_value=os.path.join(lidar_slam_share_path, 'params', "metadata_OS1_64_1024x10.json"), description="Configuration file for Ouster data to replay"),
    DeclareLaunchArgument("aggregate", default_value="false", description="Run aggregation node"),
  ])

  ##########
  ## SLAM ##
  ##########

  # Include the SLAM launch file
  slam_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(slam_launch_path),
    launch_arguments={
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'rviz': LaunchConfiguration('rviz'),
        'tags_topic': LaunchConfiguration('tags_topic'),
        'camera_topic': LaunchConfiguration('camera_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'aggregate': LaunchConfiguration('aggregate'),
    }.items()
  )

  # Ouster points conversion
  with open(os.path.join(lidar_conversion_share_path, 'params', "conversion_config.yaml"), 'r') as f:
    params_conversion = yaml.safe_load(f)['/lidar_conversions']['ros__parameters']
  params_conversion['use_sim_time'] = LaunchConfiguration("replay")

  ouster_conversion_node = Node(
    name="ouster_conversion",
    package="lidar_conversions",
    executable="ouster_conversion_node",
    output="screen",
    parameters=[params_conversion]
  )


  #####################
  ### Ouster driver ###
  #####################


  ouster_driver_path = get_package_share_directory("ouster_ros")
  ouster_parameters = os.path.join(lidar_slam_share_path, 'params', "ouster_driver_parameters.yaml")

  group_ouster = GroupAction(
    actions=[
      # Replay
      IncludeLaunchDescription(
        XMLLaunchDescriptionSource([os.path.join(ouster_driver_path, "launch", "replay.launch.xml")]),
        launch_arguments={
          "timestamp_mode"  : "TIME_FROM_INTERNAL_OSC",
          "bag_file"        : "b",
          "metadata"        : LaunchConfiguration("metadata_in"),
          "sensor_frame"    : "laser_sensor_frame",
          "laser_frame"     : "laser_data_frame",
          "imu_frame"       : "imu_data_frame",
          "viz"             : "False",
      }.items(),
        condition=IfCondition(LaunchConfiguration("replay")),
      ),
      # Live
      IncludeLaunchDescription(
        PythonLaunchDescriptionSource([os.path.join(ouster_driver_path, "launch", "driver.launch.py")]),
        launch_arguments={
          "params_file"     : LaunchConfiguration("driver_parameter_file"),
          "viz"             : "False",
      }.items(),
        condition=UnlessCondition(LaunchConfiguration("replay")),
      ),
    ],
    condition=IfCondition(LaunchConfiguration("os_driver"))
  )

  ##########
  ##  TF  ##
  ##########

  # Static TF base to Os sensor (for compatibility with ROS1 bags)
  tf_base_to_os_sensor = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_os_sensor",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "os_sensor"]
  )

  # Static TF base to laser sensor
  tf_base_to_laser_sensor = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_laser_sensor_frame",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "laser_sensor_frame"]
  )

  # Static TF laser to Os lidar (when the driver is not launched)
  # To be read from the json config file in replay
  # default are for OS1 64
  # cf. https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf
  tf_laser_to_os_lidar = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_laser_sensor_frame_to_os_lidar",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0", "--y", "0", "--z", "0.0036",
               "--roll", "0", "--pitch", "0", "--yaw", "3.14",
               "--frame-id", "laser_sensor_frame", "--child-frame-id", "os_lidar"]
  )

  # Static TF laser to IMU (when the driver is not launched)
  # To be read from the json config file in replay
  # default are for OS1 64
  # cf. https://data.ouster.io/downloads/software-user-manual/software-user-manual-v2p0.pdf
  tf_laser_to_imu = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_laser_sensor_frame_to_imu_data_frame",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0.006253", "--y", "-0.011775", "--z", "0.007645",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "laser_sensor_frame", "--child-frame-id", "imu_data_frame"]
  )

  # Static TF base to wheel (to set by the user)
  tf_base_to_wheel = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_wheel",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "wheel"]
  )

  # Static TF base to ext sensor
  tf_base_to_ext_sensor = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_ext_sensor",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "ext_sensor"]
  )

  ld.add_action(ouster_conversion_node)
  ld.add_action(group_ouster)
  ld.add_action(slam_launch)
  # TF
  ld.add_action(tf_base_to_os_sensor)
  ld.add_action(tf_base_to_laser_sensor)
  if LaunchConfiguration('os_driver') == 'false':
    ld.add_action(tf_laser_to_os_lidar)
    ld.add_action(tf_laser_to_imu)
  ld.add_action(tf_base_to_wheel)
  ld.add_action(tf_base_to_ext_sensor)
  return (ld)
