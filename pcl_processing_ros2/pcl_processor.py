import rclpy 
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from sensor_msgs.msg import PointCloud2, PointField
from stamped_std_msgs.msg import Float32Stamped
from data_gathering_msgs.srv import RequestPCLVolumeDiff

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
        self.publisher_grinded_cloud = self.create_publisher(PointCloud2,'grinded_cloud', 10)
        
        self.volume_calculation_server = self.create_service(RequestPCLVolumeDiff, 'calculate_volume_lost', self.calculate_volume_difference, callback_group=MutuallyExclusiveCallbackGroup())

        #PCL Collector
        self.mesh_filename = 'sample_plate'
        self.global_frame_id = 'base_link'


    def init_parameters(self) -> None:
        self.declare_parameter('dist_threshold',                '0.0006')    # filter for plane projection
        self.declare_parameter('plane_error_allowance',         '5'    )    # in degree
        self.declare_parameter('clusterscan_eps',               '0.002')    # size for cluster grouping in DBScan
        self.declare_parameter('cluster_neighbor',              '20'    )    # number of required neighbour points to remove outliers
        self.declare_parameter('laserline_threshold',           '0.00008')    # scan resolution line axis in m
        self.declare_parameter('feedaxis_threshold',            '0.00012')    # scan resolution feed axis in m
        self.declare_parameter('plate_thickness',               '0.002' )    # in m

        self.dist_threshold = float(self.get_parameter('dist_threshold').get_parameter_value().string_value)
        self.cluster_neighbor = int(self.get_parameter('cluster_neighbor').get_parameter_value().string_value) 
        self.plate_thickness = float(self.get_parameter('plate_thickness').get_parameter_value().string_value) 
        self.plane_error_allowance = float(self.get_parameter('plane_error_allowance').get_parameter_value().string_value)
        self.clusterscan_eps = float(self.get_parameter('clusterscan_eps').get_parameter_value().string_value)
        self.laserline_threshold = float(self.get_parameter('laserline_threshold').get_parameter_value().string_value)
        self.feedaxis_threshold = float(self.get_parameter('feedaxis_threshold').get_parameter_value().string_value)

    def calculate_volume_difference(self, request, response):
        self.get_logger().info("Volume calculation request received...")
        
        pcl1 = self.convert_ros_to_open3d(request.initial_pointcloud)
        pcl2 = self.convert_ros_to_open3d(request.final_pointcloud)
        self.plate_thickness = request.plate_thickness

        #filter point by plane and project onto it
        pcl1, mesh1_pca_basis, mesh1_plane_centroid = self.pcl_functions.filter_project_points_by_plane(pcl1, distance_threshold=self.dist_threshold)
        pcl2, mesh2_pca_basis, mesh2_plane_centroid = self.pcl_functions.filter_project_points_by_plane(pcl2, distance_threshold=self.dist_threshold)
        pcl1 = self.pcl_functions.sort_plate_cluster(pcl1, eps=0.0005, min_points=100)
        pcl2 = self.pcl_functions.sort_plate_cluster(pcl2, eps=0.0005, min_points=100)

        self.get_logger().info('PCL Projected on plane')

        # Check alignment
        cos_angle = np.dot(mesh1_pca_basis[2], mesh2_pca_basis[2]) / (np.linalg.norm(mesh1_pca_basis[2]) * np.linalg.norm(mesh2_pca_basis[2]))
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0)) * 180 / np.pi
        if abs(angle) > self.plane_error_allowance:     #10 degree misalignment throw error
            raise ValueError(f"Plane normals differ too much: {angle} degrees")

        projected_points_mesh2 = self.pcl_functions.project_points_onto_plane(np.asarray(pcl2.points), mesh1_pca_basis[2], mesh1_plane_centroid)
        pcl2.points = o3d.utility.Vector3dVector(projected_points_mesh2)

        #transform points to local xy plane
        pcl1_local = self.pcl_functions.transform_to_local_pca_coordinates(pcl1, mesh1_pca_basis, mesh1_plane_centroid )
        pcl2_local = self.pcl_functions.transform_to_local_pca_coordinates(pcl2, mesh1_pca_basis, mesh1_plane_centroid )


        self.get_logger().info('Filtering for changes in pcl')
        changed_pcl_local = self.pcl_functions.filter_missing_points_by_xy(pcl1_local, pcl2_local, x_threshold=self.feedaxis_threshold, y_threshold=self.laserline_threshold)
        
        # after sorting
        changed_pcl_local = self.pcl_functions.sort_largest_cluster(changed_pcl_local, eps=self.clusterscan_eps, min_points=self.cluster_neighbor, remove_outliers=True)

        # Check if there are any missing points detected
        if changed_pcl_local is None or len(np.asarray(changed_pcl_local.points)) == 0:
            self.get_logger().info("No detectable difference between point clouds. Lost volume is 0.")
            lost_volume = 0.0
        else: 
            #area from bounding box
            width, height, area_bb = self.pcl_functions.create_bbox_from_pcl(changed_pcl_local)
            #area from convex hull
            area, hull_convex_2d = self.pcl_functions.compute_convex_hull_area_xy(changed_pcl_local)
            self.pcl_functions.compute_concave_hull_area_xy(changed_pcl_local, hull_convex_2d)
            self.get_logger().info(f"bbox width: {width * 1000} mm, height: {height * 1000} mm")
            #self.get_logger().info(f"bbox area: {area_bb * (1000**3)} mm^2, hull_area: {area * (1000**3)} mm^2")
            lost_volume = area * self.plate_thickness
            self.get_logger().info(f"Lost Volume: {lost_volume * (1000**3)} mm^3")

        #transform back to global for visualization
        changed_pcl_global = self.pcl_functions.transform_to_global_coordinates(changed_pcl_local, mesh1_pca_basis, mesh1_plane_centroid) 

        # Prepare and publish grinded cloud and volume message
        msg_stamped = Float32Stamped()
        msg_stamped.data = float(lost_volume)
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