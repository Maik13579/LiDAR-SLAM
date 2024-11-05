from email.policy import default
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
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
    DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time when replaying rosbags with '--clock' option."),
    DeclareLaunchArgument("outdoor", default_value="true", description="Decide which set of parameters to use"),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    DeclareLaunchArgument("camera_topic", default_value="camera", description="topic from which to get the rgb camera data"),
    DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="topic from which to get the rgb camera info"),
    DeclareLaunchArgument("aggregate", default_value="false", description="Run aggregation node"),
    DeclareLaunchArgument("domain_id", default_value="0", description="Set to different value to avoid interference when several computers running ROS2 on the same network."),
    SetEnvironmentVariable(name='ROS_DOMAIN_ID',value=LaunchConfiguration('domain_id')),
  ])


  ##########
  ## SLAM ##
  ##########

  # Include the SLAM launch file
  slam_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(slam_launch_path),
    launch_arguments={
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'outdoor': LaunchConfiguration('outdoor'),
        'rviz': LaunchConfiguration('rviz'),
        'tags_topic': LaunchConfiguration('tags_topic'),
        'camera_topic': LaunchConfiguration('camera_topic'),
        'camera_info_topic': LaunchConfiguration('camera_info_topic'),
        'aggregate': LaunchConfiguration('aggregate'),
    }.items()
  )

  # Hesai points conversion
  with open(os.path.join(lidar_conversion_share_path, 'params', "conversion_config.yaml"), 'r') as f:
    params_conversion = yaml.safe_load(f)['/lidar_conversions']['ros__parameters']
  params_conversion['use_sim_time'] = LaunchConfiguration("use_sim_time")

  hesai_conversion_node = Node(
    package="lidar_conversions",
    executable="hesai_conversion_node",
    name="hesai_conversion",
    output="screen",
    parameters=[params_conversion]
  )

  
  # Static TF base to hesai LiDAR
  tf_base_to_hesai = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_lidar",
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "hesai_lidar"]
  )

  # Static TF base to wheel
  tf_base_to_wheel = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_wheel",
    parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "wheel"]
  )

  ld.add_action(slam_launch)
  ld.add_action(hesai_conversion_node)
  # TF
  ld.add_action(tf_base_to_hesai)
  ld.add_action(tf_base_to_wheel)

  return (ld)
