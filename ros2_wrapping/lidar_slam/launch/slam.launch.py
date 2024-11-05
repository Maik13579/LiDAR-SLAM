import os
import yaml
import re
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def update_rviz_topics(context, rviz_config_path, remappings):
    """
    Updates topics in an RViz config file based on provided mappings.

    Parameters:
    - context: Launch context to resolve substitutions.
    - rviz_config_path (str): Path to the RViz config file.
    - remappings (list): A list of tuples where each tuple is (original_topic, new_topic).
    """
    # Resolve LaunchConfiguration substitutions
    resolved_remappings = [
        (original, new_topic.perform(context)) for original, new_topic in remappings
    ]

    # Load the RViz config file as text
    with open(rviz_config_path, 'r') as file:
        config_data = file.read()

    # Replace each topic based on the resolved mappings
    for original_topic, new_topic in resolved_remappings:
        config_data = re.sub(rf'\b{re.escape(original_topic)}\b', new_topic, config_data)

    # Save the updated config data back to a new file
    new_file_name = rviz_config_path.replace(".rviz", "_remapped.rviz")
    with open(new_file_name, 'w') as file:
        file.write(config_data)

    return new_file_name


def launch_setup(context, *args, **kwargs):
    lidar_slam_share_path = get_package_share_directory('lidar_slam')
    rviz_config_path = os.path.join(lidar_slam_share_path, 'params', 'slam.rviz')

    remappings = [
        ("lidar_points", LaunchConfiguration("lidar_points_topic")),
        ("slam_command", LaunchConfiguration("slam_command_topic")),
        ("slam_confidence", LaunchConfiguration("slam_confidence_topic")),
        ("slam_odom", LaunchConfiguration("slam_odom_topic")),
        ("slam_predicted_odom", LaunchConfiguration("slam_predicted_odom_topic")),
        ("slam_registered_points", LaunchConfiguration("slam_registered_points_topic")),
        ("aggregated_cloud", LaunchConfiguration("aggregated_cloud_topic")),
        ("tag_detections", LaunchConfiguration("tags_topic")),
        ("camera", LaunchConfiguration("camera_topic")),
        ("camera_info", LaunchConfiguration("camera_info_topic")),
        ("clicked_point", LaunchConfiguration("clicked_point_topic")),
        ("set_slam_pose", LaunchConfiguration("set_slam_pose_topic")),
        # Topic Keypoints
        ("keypoints/edges", LaunchConfiguration("keypoints_edges_topic")),
        ("keypoints/intensity_edges", LaunchConfiguration("keypoints_intensity_edges_topic")),
        ("keypoints/planes", LaunchConfiguration("keypoints_planes_topic")),
        ("keypoints/blobs", LaunchConfiguration("keypoints_blobs_topic")),
        # Topic Submaps
        ("submaps/edges", LaunchConfiguration("submaps_edges_topic")),
        ("submaps/intensity_edges", LaunchConfiguration("submaps_intensity_edges_topic")),
        ("submaps/planes", LaunchConfiguration("submaps_planes_topic")),
        ("submaps/blobs", LaunchConfiguration("submaps_blobs_topic")),
        # Topics Maps
        ("maps/edges", LaunchConfiguration("maps_edges_topic")),
        ("maps/intensity_edges", LaunchConfiguration("maps_intensity_edges_topic")),
        ("maps/planes", LaunchConfiguration("maps_planes_topic")),
        ("maps/blobs", LaunchConfiguration("maps_blobs_topic")),
        # Topics Obstacles
        ("obstacles/bboxes", LaunchConfiguration("obstacles_bboxes_topic")),
        ("obstacles/occupancy_grid", LaunchConfiguration("obstacles_occupancy_grid_topic")),
        # Services
        ("/lidar_slam/save_pc", LaunchConfiguration("save_pc_service")),
        ("/lidar_slam/reset", LaunchConfiguration("reset_service")),
    ]

    # Update RViz config file and get the path to the modified file
    new_rviz_config_path = update_rviz_topics(context, rviz_config_path, remappings)
    
    # Define the RViz Node with the updated config
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", new_rviz_config_path],
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'slam_control_pannel.slam_command_topic': LaunchConfiguration('slam_command_topic'),
            'slam_control_pannel.save_pc_service': LaunchConfiguration('save_pc_service'),
            'slam_control_pannel.reset_service': LaunchConfiguration('reset_service'),
            'slam_control_pannel.slam_confidence_topic': LaunchConfiguration('slam_confidence_topic'),
        }],
        condition=IfCondition(LaunchConfiguration("rviz"))
    )

    # Outdoor Lidar SLAM node
    with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_outdoor.yaml"), 'r') as f:
        params_slam_out = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
    params_slam_out['use_sim_time'] = LaunchConfiguration("use_sim_time")

    slam_outdoor_node = Node(
        name="lidar_slam",
        package="lidar_slam",
        executable="lidar_slam_node",
        output="screen",
        parameters=[params_slam_out],
        remappings=remappings,
        condition=IfCondition(LaunchConfiguration("outdoor"))
    )

    # Indoor Lidar SLAM node
    with open(os.path.join(lidar_slam_share_path, 'params', "slam_config_indoor.yaml"), 'r') as f:
        params_slam_in = yaml.safe_load(f)['/lidar_slam']['ros__parameters']
    params_slam_in['use_sim_time'] = LaunchConfiguration("use_sim_time")

    slam_indoor_node = Node(
        name="lidar_slam",
        package="lidar_slam",
        executable="lidar_slam_node",
        output="screen",
        parameters=[params_slam_in],
        remappings=remappings,
        condition=UnlessCondition(LaunchConfiguration("outdoor"))
    )

    # Aggregate points
    with open(os.path.join(lidar_slam_share_path, 'params', "aggregation_config.yaml"), 'r') as f:
        params_aggregation = yaml.safe_load(f)['/aggregation']['ros__parameters']
    params_aggregation['use_sim_time'] = LaunchConfiguration("use_sim_time")

    aggregation_node = Node(
        name="aggregation",
        package="lidar_slam",
        executable="aggregation_node",
        output="screen",
        parameters=[params_aggregation],
        remappings=remappings,
        condition=IfCondition(LaunchConfiguration("aggregate"))
    )

    return [rviz_node, slam_outdoor_node, slam_indoor_node, aggregation_node]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time when replaying rosbags with '--clock' option."),
        DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
        DeclareLaunchArgument("outdoor", default_value="true", description="Use outdoor configuration for SLAM"),
        DeclareLaunchArgument("aggregate", default_value="false", description="Run aggregation node"),

        # Topics
        DeclareLaunchArgument("lidar_points_topic", default_value="lidar_points", description="Topic from which to get the point cloud"),
        DeclareLaunchArgument("slam_command_topic", default_value="slam/command", description="Topic to which to subscribe the SLAM command"),
        DeclareLaunchArgument("slam_confidence_topic", default_value="slam/confidence", description="Topic to which to publish the SLAM confidence"),
        DeclareLaunchArgument("slam_odom_topic", default_value="slam/odom", description="Topic to which to publish the SLAM odometry"),
        DeclareLaunchArgument("slam_predicted_odom_topic", default_value="slam/predicted_odom", description="Topic to which to publish the SLAM predicted odometry"),
        DeclareLaunchArgument("slam_registered_points_topic", default_value="slam/registered_points", description="Topic to which to publish the SLAM registered points"),
        DeclareLaunchArgument("aggregated_cloud_topic", default_value="slam/aggregated_cloud", description="Topic to which to publish the aggregated point cloud"),
        DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
        DeclareLaunchArgument("camera_topic", default_value="camera", description="Topic from which to get the RGB camera data"),
        DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="Topic from which to get the RGB camera info"),
        DeclareLaunchArgument("clicked_point_topic", default_value="clicked_point", description="Topic from which to get the clicked point"),
        DeclareLaunchArgument("set_slam_pose_topic", default_value="slam/set_slam_pose", description="Topic from which to set the SLAM pose"),

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
        DeclareLaunchArgument("save_pc_service", default_value="/slam/save_pc", description="Service to save pointcloud"),
        DeclareLaunchArgument("reset_service", default_value="/slam/reset", description="Service to reset SLAM"),

        # Launch OpaqueFunction to resolve substitutions before setup
        OpaqueFunction(function=launch_setup)
    ])
