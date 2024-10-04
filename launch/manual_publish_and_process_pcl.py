import os
from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration,PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch_ros.actions import Node


def generate_launch_description():
    pkg = "pcl_processing_ros2"
    

    # Manually publish pointcloud
    manual_cloud_publish = Node(
        package=pkg,
        executable="pcl_publish_manual",
        name='cloud_publisher',    # This is the first line in the config file 
    )

    #combined cloud listener / processor
    combine_cloud_listener = Node(
        package=pkg,
        executable="pcl_processor",
        name='pcl_process',    # This is the first line in the config file 
        parameters=[
            {'distance_filter_threshold':       '0.0003',            # Distance to filter grinded area. do not go near #50-80 micron on line axis, 200 micron on feed axis of rate 30 second
            'neighbor_threshold':               '10',                # filter outlier with #of neighbour point threshold
            'plate_thickness':                  '2',                # in mm
            }
        ]
    )

    ld = LaunchDescription()
    ld.add_action(manual_cloud_publish)
    ld.add_action(combine_cloud_listener)

    return ld