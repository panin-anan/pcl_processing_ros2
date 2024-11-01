import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    # Define LaunchConfigurations for each parameter
    dist_threshold = LaunchConfiguration('dist_threshold')
    cluster_neighbor = LaunchConfiguration('cluster_neighbor')
    plane_error_allowance = LaunchConfiguration('plane_error_allowance')
    clusterscan_eps = LaunchConfiguration('clusterscan_eps')
    laserline_threshold = LaunchConfiguration('laserline_threshold')
    feedaxis_threshold = LaunchConfiguration('feedaxis_threshold')
    concave_resolution = LaunchConfiguration('concave_resolution')
    filter_down_size = LaunchConfiguration('filter_down_size')
    belt_width_threshold = LaunchConfiguration('belt_width_threshold')

    # Define the node with parameters passed as LaunchConfigurations
    combine_cloud_listener = Node(
        package="pcl_processing_ros2",
        executable="pcl_processor",
        name='pcl_process',
        parameters=[{
            'dist_threshold': dist_threshold,
            'cluster_neighbor': cluster_neighbor,
            'plane_error_allowance': plane_error_allowance,
            'clusterscan_eps': clusterscan_eps,
            'laserline_threshold': laserline_threshold,
            'feedaxis_threshold': feedaxis_threshold,
            'concave_resolution': concave_resolution,
            'filter_down_size': filter_down_size,
            'belt_width_threshold': belt_width_threshold,
        }]
    )

    # Declare launch arguments with default values for each parameter
    ld = LaunchDescription([
        DeclareLaunchArgument('dist_threshold', default_value='0.0006', description='Distance threshold for filtering'),
        DeclareLaunchArgument('cluster_neighbor', default_value='20', description='Cluster neighbor count threshold'),
        DeclareLaunchArgument('plane_error_allowance', default_value='5', description='Plane error allowance in degrees'),
        DeclareLaunchArgument('clusterscan_eps', default_value='0.00025', description='Cluster scan epsilon value'),
        DeclareLaunchArgument('laserline_threshold', default_value='0.00008', description='Laser line resolution threshold'),
        DeclareLaunchArgument('feedaxis_threshold', default_value='0.00012', description='Feed axis resolution threshold'),
        DeclareLaunchArgument('concave_resolution', default_value='0.0005', description='Concave hull resolution'),
        DeclareLaunchArgument('filter_down_size', default_value='0.0002', description='Downsampling size for clustering'),
        DeclareLaunchArgument('belt_width_threshold', default_value='0.8', description='Minimum width threshold for valid volume removal'),
        
        # Add the node to the launch description
        combine_cloud_listener
    ])

    return ld
