import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import PointCloud2, PointField
from stamped_std_msgs.msg import Float32Stamped
from std_msgs.msg import Header
from data_gathering_msgs.srv import RequestPCLVolumeDiff
from visualization_msgs.msg import Marker
from geometry_msgs.msg import Point

import open3d as o3d
import numpy as np
import os

from pcl_processing_ros2.pcl_functions import PCLfunctions


class PCLprocessor(Node):
    def __init__(self):
        super().__init__('pcl_processor')
        self.init_parameters()
        
        #initialize PCLfunctions
        self.pcl_functions = PCLfunctions()

        #Publisher to publish volume lost
        self.publisher_volume = self.create_publisher(Float32Stamped, '/scanner/volume', 10)
        self.publisher_grinded_cloud = self.create_publisher(PointCloud2,'/grinded_cloud', 10)
        self.publisher_hull_lines = self.create_publisher(Marker, '/concave_hull_lines', 10)  # Publisher for concave hull lines

        self.volume_calculation_server = self.create_service(RequestPCLVolumeDiff, 'calculate_volume_lost', self.calculate_volume_difference, callback_group=MutuallyExclusiveCallbackGroup())

        #PCL Collector
        self.mesh_filename = 'sample_plate'
        self.global_frame_id = 'base_link'


    def init_parameters(self) -> None:
        self.declare_parameter('dist_threshold',                '0.0006')    # filter for plane projection
        self.declare_parameter('plane_error_allowance',         '5'    )    # in degree
        self.declare_parameter('clusterscan_eps',               '0.00025')    # size for cluster grouping in DBScan
        self.declare_parameter('cluster_neighbor',              '20'    )    # number of required neighbour points to remove outliers
        self.declare_parameter('laserline_threshold',           '0.00008')    # scan resolution line axis in m
        self.declare_parameter('feedaxis_threshold',            '0.00012')    # scan resolution feed axis in m
        self.declare_parameter('plate_thickness',               '0.002' )    # in m
        self.declare_parameter('concave_resolution',            '0.0005')   # in m
        self.declare_parameter('filter_down_size',              '0.0002')   #   in m
        self.declare_parameter('belt_width_threshold',          '0.020')    # in m

        self.dist_threshold = float(self.get_parameter('dist_threshold').get_parameter_value().string_value)
        self.cluster_neighbor = int(self.get_parameter('cluster_neighbor').get_parameter_value().string_value) 
        self.plate_thickness = float(self.get_parameter('plate_thickness').get_parameter_value().string_value) 
        self.plane_error_allowance = float(self.get_parameter('plane_error_allowance').get_parameter_value().string_value)
        self.clusterscan_eps = float(self.get_parameter('clusterscan_eps').get_parameter_value().string_value)
        self.laserline_threshold = float(self.get_parameter('laserline_threshold').get_parameter_value().string_value)
        self.feedaxis_threshold = float(self.get_parameter('feedaxis_threshold').get_parameter_value().string_value)
        self.concave_resolution = float(self.get_parameter('concave_resolution').get_parameter_value().string_value)
        self.filter_down_size = float(self.get_parameter('filter_down_size').get_parameter_value().string_value)
        self.belt_width_threshold = float(self.get_parameter('belt_width_threshold').get_parameter_value().string_value)

        self.settings = {'dist_threshold':       self.dist_threshold,
                        'plane_error_allowance':      self.cluster_neighbor,
                        'clusterscan_eps':           self.plate_thickness,
                        'cluster_neighbor':        self.plane_error_allowance,
                        'laserline_threshold':      self.clusterscan_eps,
                        'feedaxis_threshold':      self.laserline_threshold,
                        'plate_thickness':         self.feedaxis_threshold,
                        'concave_resolution':      self.concave_resolution,
                        'filter_down_size':       self.filter_down_size,
                        'belt_width_threshold':      self.belt_width_threshold
        }

    def calculate_volume_difference(self, request, response):
        self.get_logger().info("Volume calculation request received...")
        
        pcl1 = self.convert_ros_to_open3d(request.initial_pointcloud)
        pcl2 = self.convert_ros_to_open3d(request.final_pointcloud)
        self.plate_thickness = request.plate_thickness

        
        result = self.pcl_functions.calculate_volume_difference(pcl1, pcl2, self.plate_thickness, self.settings, self.get_logger())
        # Early return because the volume difference could not be calculated
        if result is None:
            return response
        
        lost_volume, changed_pcl_global, hull_cloud_global = result
        
        hull_lines_msg = self.create_hull_lines_marker(np.asarray(hull_cloud_global.points))
        self.publisher_hull_lines.publish(hull_lines_msg)

        # Prepare and publish grinded cloud and volume message
        msg_stamped = Float32Stamped(data=float(lost_volume) * 1000**3, header=Header(stamp=self.get_clock().now().to_msg()))
        self.publisher_volume.publish(msg_stamped)

        diff_pcl_global = self.create_pcl_msg(changed_pcl_global)
        self.publisher_grinded_cloud.publish(diff_pcl_global) 

        response.volume_difference = lost_volume
        response.difference_pointcloud = diff_pcl_global
        return response 

    def create_pcl_msg(self, o3d_pcl):
        datapoints = np.asarray(o3d_pcl.points, dtype=np.float32)

        pointcloud = PointCloud2()
        pointcloud.header.stamp = self.get_clock().now().to_msg()
        pointcloud.header.frame_id = self.global_frame_id

        dims = ['x', 'y', 'z']

        bytes_per_point = 4
        fields = [PointField(name=direction, offset=i * bytes_per_point, datatype=PointField.FLOAT32, count=1) for i, direction in enumerate(dims)]
        pointcloud.fields = fields
        pointcloud.point_step = len(fields) * bytes_per_point
        total_points = datapoints.shape[0]
        pointcloud.is_dense = False
        pointcloud.height = 1
        pointcloud.width = total_points

        pointcloud.data = datapoints.flatten().tobytes()
        return pointcloud

    def convert_ros_to_open3d(self, pcl_msg):
        # Extract the point cloud data from the ROS2 message
        loaded_array = np.frombuffer(pcl_msg.data, dtype=np.float32).reshape(-1, len(pcl_msg.fields)) 
        o3d_pcl = o3d.geometry.PointCloud()
        o3d_pcl.points = o3d.utility.Vector3dVector(loaded_array)
        
        self.get_logger().info('Converted combined pointcloud for open3d')
        return o3d_pcl

    def create_hull_lines_marker(self, hull_points):
        """Helper function to create a Marker message with lines connecting hull points"""
        marker = Marker()
        marker.header.frame_id = self.global_frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "concave_hull"
        marker.id = 0
        marker.type = Marker.LINE_STRIP
        marker.action = Marker.ADD
        marker.pose.orientation.w = 1.0

        # Add points from hull to marker
        for point in hull_points:
            p = Point()
            p.x = point[0]
            p.y = point[1]
            p.z = point[2]  # Set to 0 if the points are 2D or adjust as necessary
            marker.points.append(p)

        # Connect the last point to the first to close the loop
        marker.points.append(marker.points[0])

        # Set line width and color
        marker.scale.x = 0.00005  # Line width
        marker.color.a = 1.0   # Alpha (transparency)
        marker.color.r = 1.0   # Red color

        return marker

def main(args=None):
    rclpy.init(args=args)
    pcl_processor = PCLprocessor()
    executor = MultiThreadedExecutor()

    try:
        rclpy.spin(pcl_processor, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        pcl_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()