
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')
  slam_launch_path = os.path.join(lidar_slam_share_path, 'launch', 'slam.launch.py')
  lidar_conversion_share_path = get_package_share_directory('lidar_conversions')

  ld = LaunchDescription([
    # General args
    DeclareLaunchArgument("replay", default_value="true", description="Whether to process live or replayed data"),
    DeclareLaunchArgument("pointcloud2", default_value="false" , description="True if pointcloud message is in pointcloud2 format"),
    DeclareLaunchArgument("outdoor", default_value="true", description="Decide which set of parameters to use."),
    DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
    DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
    DeclareLaunchArgument("camera_topic", default_value="camera", description="topic from which to get the rgb camera data"),
    DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="topic from which to get the rgb camera info"),
    DeclareLaunchArgument("aggregate", default_value="false", description="run aggregation node"),
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

  # Conversion node
  livox_conversion_node = Node(
      name="livox_conversion_node",
      package="lidar_conversions",
      executable="livox_conversion_node",
      output="screen",
      parameters=[{"pointcloud2" :LaunchConfiguration("pointcloud2")}]
  )

  # Static TF base to livox LiDAR
  tf_base_to_livox = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    name="tf_base_to_lidar",
    parameters=[{'use_sim_time': LaunchConfiguration('replay')}],
    arguments=["--x", "0", "--y", "0", "--z", "0",
               "--roll", "0", "--pitch", "0", "--yaw", "0",
               "--frame-id", "base_link", "--child-frame-id", "livox_frame"]
  )

  # Static TF base to wheel
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

  ld.add_action(slam_launch)
  ld.add_action(livox_conversion_node)

  # TF
  ld.add_action(tf_base_to_livox)
  ld.add_action(tf_base_to_wheel)
  ld.add_action(tf_base_to_ext_sensor)

  return (ld)