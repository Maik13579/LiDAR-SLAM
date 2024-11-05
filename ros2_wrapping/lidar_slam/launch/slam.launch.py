from email.policy import default
import os
import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    lidar_slam_share_path = get_package_share_directory('lidar_slam')

    # Declare SLAM-specific arguments
    ld = LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time when replaying rosbags with '--clock' option."),
        DeclareLaunchArgument("rviz", default_value="true", description="Visualize results with RViz."),
        DeclareLaunchArgument("outdoor", default_value="true", description="Use outdoor configuration for SLAM"),
        DeclareLaunchArgument("tags_topic", default_value="tag_detections", description="Topic from which to get the tag measurements"),
        DeclareLaunchArgument("camera_topic", default_value="camera", description="Topic from which to get the RGB camera data"),
        DeclareLaunchArgument("camera_info_topic", default_value="camera_info", description="Topic from which to get the RGB camera info"),
        DeclareLaunchArgument("aggregate", default_value="false", description="Run aggregation node"),
    ])

    ##########
    ## Rviz ##
    ##########
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", os.path.join(lidar_slam_share_path, 'params', 'slam.rviz')],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')},],
        condition = IfCondition(LaunchConfiguration("rviz"))
    )

    ###########
    ## SLAM  ##
    ###########
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
        remappings=[("tag_detections", LaunchConfiguration("tags_topic")),
                    ("camera", LaunchConfiguration("camera_topic")),
                    ("camera_info", LaunchConfiguration("camera_info_topic")),],
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
        remappings=[("tag_detections", LaunchConfiguration("tags_topic")),
                    ("camera", LaunchConfiguration("camera_topic")),
                    ("camera_info", LaunchConfiguration("camera_info_topic")),],
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
        condition=IfCondition(LaunchConfiguration("aggregate"))
    )

    # Add nodes to launch description
    ld.add_action(rviz_node)
    ld.add_action(slam_outdoor_node)
    ld.add_action(slam_indoor_node)
    ld.add_action(aggregation_node)

    return ld
