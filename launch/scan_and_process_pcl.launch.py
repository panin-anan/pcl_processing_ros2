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
        parameters=[
            {'dist_threshold':       '0.0006',            # Distance to filter grinded area. do not go near #50-80 micron on line axis, 200 micron on feed axis of rate 30 second
            'cluster_neighbor':               '20',                # filter outlier with #of neighbour point threshold
            'plate_thickness':                  '0.0023',                # in m
            'plane_error_allowance':            '5',               #in degree
            'clusterscan_eps':               '0.00025',              # cluster minimum dist grouping in m
            'laserline_threshold':           '0.00008',      # scan resolution line axis in m
            'feedaxis_threshold':            '0.00012',     # scan resolution robot feed axis in m
            'concave_resolution':             '0.0005',       #concave hull resolution
            'filter_down_size':                 '0.0002',       #voxel down size on clustering
            'belt_width_threshold':             '0.04',           #minimum detected belt width in m 
            }
        ]
    )

    # UR Trajectory Controller
    '''
    trajectory_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            [os.path.join(get_package_share_directory('ur_trajectory_controller'),'launch','ur_surface_measurement.launch.py')])
    )
    '''
    ld = LaunchDescription()
    ld.add_action(combine_cloud_listener)
    #ld.add_action(trajectory_control)

    return ld