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

    def calculate_volume_difference(self, request, response):
        self.get_logger().info("Volume calculation request received...")
        
        pcl1 = self.convert_ros_to_open3d(request.initial_pointcloud)
        pcl2 = self.convert_ros_to_open3d(request.final_pointcloud)
        self.plate_thickness = request.plate_thickness

        #filter point by plane and project onto it
        pcl1, mesh1_pca_basis, mesh1_plane_centroid = self.pcl_functions.filter_project_points_by_plane(pcl1, distance_threshold=self.dist_threshold)
        pcl2, mesh2_pca_basis, mesh2_plane_centroid = self.pcl_functions.filter_project_points_by_plane(pcl2, distance_threshold=self.dist_threshold)
        pcl1_plane = pcl1
        pcl2_plane = pcl2
        pcl1 = self.pcl_functions.sort_plate_cluster(pcl1_plane, eps=0.001, min_points=30, use_downsampling=True, downsample_voxel_size=self.filter_down_size)
        pcl2 = self.pcl_functions.sort_plate_cluster(pcl2_plane, eps=0.001, min_points=30, use_downsampling=True, downsample_voxel_size=self.filter_down_size)

        # Check if the largest cluster has at least half the points of the original point cloud
        if len(pcl1.points) < len(pcl1_plane.points) / 2:
            self.get_lgoger().info(f"voxel down algorithm failed. Resorting with original pcl")
            pcl1 = self.pcl_functions.sort_plate_cluster(pcl1_plane, eps=0.0005, min_points=30, use_downsampling=False)
            pcl2 = self.pcl_functions.sort_plate_cluster(pcl2_plane, eps=0.0005, min_points=30, use_downsampling=False)

        self.get_logger().info('PCL Projected on plane')

        # Check alignment
        cos_angle = np.dot(mesh1_pca_basis[2], mesh2_pca_basis[2]) / (np.linalg.norm(mesh1_pca_basis[2]) * np.linalg.norm(mesh2_pca_basis[2]))
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0)) * 180 / np.pi
        if abs(angle) > self.plane_error_allowance:     #10 degree misalignment throw error
            self.get_logger().error(f"Plane normals differ too much: {angle} degrees. Something may have moved in between recording the two pointclouds, or the wrong plane was detected for one or more of the pointclouds.")
            return response 

        projected_points_mesh2 = self.pcl_functions.project_points_onto_plane(np.asarray(pcl2.points), mesh1_pca_basis[2], mesh1_plane_centroid)
        pcl2.points = o3d.utility.Vector3dVector(projected_points_mesh2)

        #transform points to local xy plane
        pcl1_local = self.pcl_functions.transform_to_local_pca_coordinates(pcl1, mesh1_pca_basis, mesh1_plane_centroid )
        pcl2_local = self.pcl_functions.transform_to_local_pca_coordinates(pcl2, mesh1_pca_basis, mesh1_plane_centroid )


        self.get_logger().info('Filtering for changes in pcl')
        changed_pcl_local = self.pcl_functions.filter_missing_points_by_xy(pcl1_local, pcl2_local, x_threshold=self.feedaxis_threshold, y_threshold=self.laserline_threshold)
        changed_pcl_local_all = changed_pcl_local
        # after sorting
        changed_pcl_local = self.pcl_functions.sort_largest_cluster(changed_pcl_local, eps=self.clusterscan_eps, min_points=self.cluster_neighbor, remove_outliers=True)

        # Check if there are any missing points detected
        if changed_pcl_local is None or len(np.asarray(changed_pcl_local.points)) == 0:
            self.get_logger().info("No detectable difference between point clouds. Lost volume is 0.")
            lost_volume = 0.0
        else: 
            #area from bounding box
            height, width, area_bb = self.pcl_functions.create_bbox_from_pcl(changed_pcl_local)
            if width < self.belt_width_threshold:
                self.get_logger().info("clustering not appropriate. increasing threshold")
                changed_pcl_local = self.pcl_functions.sort_largest_cluster(changed_pcl_local_all, eps=self.clusterscan_eps*5, min_points=self.cluster_neighbor, remove_outliers=True)
            #area from convex hull
            area, hull_convex_2d = self.pcl_functions.compute_convex_hull_area_xy(changed_pcl_local)
            area_concave, hull_concave_2d_cloud = self.pcl_functions.compute_concave_hull_area_xy(changed_pcl_local, hull_convex_2d, concave_resolution= self.concave_resolution)
            self.get_logger().info(f"bbox width: {width * 1000} mm, height: {height * 1000} mm")
            self.get_logger().info(f"bbox area: {area_bb * (1000**3)} mm^2, convex_hull_area: {area * (1000**3)} mm^2, concave_hull_area: {area_concave * (1000**3)} mm^2")
            lost_volume = area * self.plate_thickness
            self.get_logger().info(f"Lost Volume: {lost_volume * (1000**3)} mm^3")
            hull_cloud_global = self.pcl_functions.transform_to_global_coordinates(hull_concave_2d_cloud, mesh1_pca_basis, mesh1_plane_centroid)
            hull_lines_msg = self.create_hull_lines_marker(np.asarray(hull_cloud_global.points))
            self.publisher_hull_lines.publish(hull_lines_msg)
        
        #transform back to global for visualization
        changed_pcl_global = self.pcl_functions.transform_to_global_coordinates(changed_pcl_local, mesh1_pca_basis, mesh1_plane_centroid) 

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