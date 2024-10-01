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
    
    combine_cloud_listener = Node(
        package=pkg,
        executable="pcl_processor",
        name='pcl_process',    # This is the first line in the config file 
    )

    '''
    # UR Trajectory Controller

    trajectory_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory(pkg),'launch','ur_surface_measurement.launch.py')])
    )
    '''
    ld = LaunchDescription()
    ld.add_action(combine_cloud_listener)
    #ld.add_action(trajectory_control)

    return ld