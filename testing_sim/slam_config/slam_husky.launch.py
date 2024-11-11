import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import TextSubstitution, LaunchConfiguration

def generate_launch_description():

  lidar_slam_share_path = get_package_share_directory('lidar_slam')
  slam_launch_path = os.path.join(lidar_slam_share_path, 'launch', 'slam.launch.py')
  lidar_conversion_share_path = get_package_share_directory('lidar_conversions')

  ###############
  ## ARGUMENTS ##
  ###############
  ld = LaunchDescription([
        DeclareLaunchArgument("namespace", default_value="husky", description="Namespace for the SLAM node"),
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time when replaying rosbags with '--clock' option."),
        DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
        DeclareLaunchArgument("aggregate", default_value="true", description="Run aggregation node"),
        DeclareLaunchArgument("config_filepath", default_value='/slam_config/slam.yaml', description="Path to the SLAM config file"),
        DeclareLaunchArgument("aggregation_config_filepath", default_value='/slam_config/aggregation.yaml', description="Path to the aggregation config file"),

        # Topics
        DeclareLaunchArgument("slam_command_topic", default_value="slam/command", description="Topic to which to subscribe the SLAM command"),
        DeclareLaunchArgument("slam_confidence_topic", default_value="slam/confidence", description="Topic to which to publish the SLAM confidence"),
        DeclareLaunchArgument("slam_odom_topic", default_value="slam/odom", description="Topic to which to publish the SLAM odometry"),
        DeclareLaunchArgument("slam_predicted_odom_topic", default_value="slam/predicted_odom", description="Topic to which to publish the SLAM predicted odometry"),
        DeclareLaunchArgument("slam_registered_points_topic", default_value="slam/registered_points", description="Topic to which to publish the SLAM registered points"),
        DeclareLaunchArgument("aggregated_cloud_topic", default_value="slam/aggregated_cloud", description="Topic to which to publish the aggregated point cloud"),
        DeclareLaunchArgument("clicked_point_topic", default_value="slam/clicked_point", description="Topic from which to get the clicked point"),
        DeclareLaunchArgument("set_slam_pose_topic", default_value="slam/set_slam_pose", description="Topic from which to set the SLAM pose"),

        # Sensor Topics
        DeclareLaunchArgument("lidar_points_topic", default_value="lidar_points", description="Topic from which to get the point cloud"),
        DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
        DeclareLaunchArgument("camera_topic", default_value="camera", description="Topic from which to get the RGB camera data"),
        DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="Topic from which to get the RGB camera info"),


        # Topic Keypoints
        DeclareLaunchArgument("keypoints_edges_topic", default_value="slam/keypoints/edges", description="Topic from which to get the keypoints edges"),
        DeclareLaunchArgument("keypoints_intensity_edges_topic", default_value="slam/keypoints/intensity_edges", description="Topic from which to get the keypoints intensity edges"),
        DeclareLaunchArgument("keypoints_planes_topic", default_value="slam/keypoints/planes", description="Topic from which to get the keypoints planes"),
        DeclareLaunchArgument("keypoints_blobs_topic", default_value="slam/keypoints/blobs", description="Topic from which to get the keypoints blobs"),
        # Topic Submaps
        DeclareLaunchArgument("submaps_edges_topic", default_value="slam/submaps/edges", description="Topic from which to get the submaps edges"),
        DeclareLaunchArgument("submaps_intensity_edges_topic", default_value="slam/submaps/intensity_edges", description="Topic from which to get the submaps intensity edges"),
        DeclareLaunchArgument("submaps_planes_topic", default_value="slam/submaps/planes", description="Topic from which to get the submaps planes"),
        DeclareLaunchArgument("submaps_blobs_topic", default_value="slam/submaps/blobs", description="Topic from which to get the submaps blobs"),
        # Topic Maps
        DeclareLaunchArgument("maps_edges_topic", default_value="slam/maps/edges", description="Topic from which to get the maps edges"),
        DeclareLaunchArgument("maps_intensity_edges_topic", default_value="slam/maps/intensity_edges", description="Topic from which to get the maps intensity edges"),
        DeclareLaunchArgument("maps_planes_topic", default_value="slam/maps/planes", description="Topic from which to get the maps planes"),
        DeclareLaunchArgument("maps_blobs_topic", default_value="slam/maps/blobs", description="Topic from which to get the maps blobs"),
        # Topic Obstacles
        DeclareLaunchArgument("obstacles_bboxes_topic", default_value="slam/obstacles/bboxes", description="Topic from which to get the obstacles bounding boxes"),
        DeclareLaunchArgument("obstacles_occupancy_grid_topic", default_value="slam/obstacles/occupancy_grid", description="Topic from which to get the obstacles occupancy grid"),

        # Services
        DeclareLaunchArgument("save_pc_service", default_value="slam/save_pc", description="Service to save pointcloud"),
        DeclareLaunchArgument("reset_service", default_value="slam/reset", description="Service to reset SLAM"),

  ])

  ##########
  ## SLAM ##
  ##########

  # Include the SLAM launch file
  slam_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(slam_launch_path),
    launch_arguments={
        'namespace': LaunchConfiguration('namespace'),
        'use_sim_time': LaunchConfiguration('use_sim_time'),
        'rviz': LaunchConfiguration('rviz'),
        'aggregate': LaunchConfiguration('aggregate'),
        'config_filepath' : LaunchConfiguration('config_filepath'),
        'aggregation_config_filepath' : LaunchConfiguration('aggregation_config_filepath'),
        'slam_command_topic' : LaunchConfiguration('slam_command_topic'),
        'slam_confidence_topic' : LaunchConfiguration('slam_confidence_topic'),
        'slam_odom_topic' : LaunchConfiguration('slam_odom_topic'),
        'slam_predicted_odom_topic' : LaunchConfiguration('slam_predicted_odom_topic'),
        'slam_registered_points_topic' : LaunchConfiguration('slam_registered_points_topic'),
        'aggregated_cloud_topic' : LaunchConfiguration('aggregated_cloud_topic'),
        'clicked_point_topic' : LaunchConfiguration('clicked_point_topic'),
        'set_slam_pose_topic' : LaunchConfiguration('set_slam_pose_topic'),
        'lidar_points_topic' : LaunchConfiguration('lidar_points_topic'),
        'tags_topic' : LaunchConfiguration('tags_topic'),
        'camera_topic' : LaunchConfiguration('camera_topic'),
        'camera_info_topic' : LaunchConfiguration('camera_info_topic'),
        'keypoints_edges_topic' : LaunchConfiguration('keypoints_edges_topic'),
        'keypoints_intensity_edges_topic' : LaunchConfiguration('keypoints_intensity_edges_topic'),
        'keypoints_planes_topic' : LaunchConfiguration('keypoints_planes_topic'),
        'keypoints_blobs_topic' : LaunchConfiguration('keypoints_blobs_topic'),
        'submaps_edges_topic' : LaunchConfiguration('submaps_edges_topic'),
        'submaps_intensity_edges_topic' : LaunchConfiguration('submaps_intensity_edges_topic'),
        'submaps_planes_topic' : LaunchConfiguration('submaps_planes_topic'),
        'submaps_blobs_topic' : LaunchConfiguration('submaps_blobs_topic'),
        'maps_edges_topic' : LaunchConfiguration('maps_edges_topic'),
        'maps_intensity_edges_topic' : LaunchConfiguration('maps_intensity_edges_topic'),
        'maps_planes_topic' : LaunchConfiguration('maps_planes_topic'),
        'maps_blobs_topic' : LaunchConfiguration('maps_blobs_topic'),
        'obstacles_bboxes_topic' : LaunchConfiguration('obstacles_bboxes_topic'),
        'obstacles_occupancy_grid_topic' : LaunchConfiguration('obstacles_occupancy_grid_topic'),
        'save_pc_service' : LaunchConfiguration('save_pc_service'),
        'reset_service' : LaunchConfiguration('reset_service'),
        
    }.items()
  )

  # Velodyne points conversion node
  with open(os.path.join(lidar_conversion_share_path, 'params', "conversion_config.yaml"), 'r') as f:
      params_conversion = yaml.safe_load(f)['/lidar_conversions']['ros__parameters']
  params_conversion['use_sim_time'] = LaunchConfiguration("use_sim_time")

  velodyne_conversion_node = Node(
      package="lidar_conversions",
      executable="velodyne_conversion_node",
      name="velodyne_conversion",
      output="screen",
      namespace=LaunchConfiguration("namespace"),
      parameters=[params_conversion],
      remappings=[
         ('velodyne_points', 'sensors/lidar3d_0/points_with_time'),
         ('lidar_points', LaunchConfiguration('lidar_points_topic')),
      ]
  )

  ld.add_action(velodyne_conversion_node)
  ld.add_action(slam_launch)

  return ld
