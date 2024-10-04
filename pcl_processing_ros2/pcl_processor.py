import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header, Empty
from sensor_msgs.msg import PointCloud2, PointField
from stamped_std_msgs.msg import Float32Stamped
from sensor_msgs_py import point_cloud2

import open3d as o3d
import numpy as np
import threading
import os
import copy
from datetime import datetime


from pcl_processing_ros2.mesh_calculations import (
    filter_project_points_by_plane,
    filter_missing_points_by_yz,
    create_bbox_from_pcl,
    project_points_onto_plane,
    sort_largest_cluster
)


class PCLprocessor(Node):
    def __init__(self):
        super().__init__('pcl_processor')
        self.init_parameters()
        
        # Subscriber for the combined point cloud
        self.combinepcl_subscription = self.create_subscription(PointCloud2,'combined_cloud',self.combined_cloud_callback,10)

        #Publisher to publish volume lost
        self.publisher_volume = self.create_publisher(Float32Stamped, '/scanner/volume', 10)

        # Pointcloud storage
        self.combined_pcl_initial = []
        self.combined_pcl_postgrind = []
        self.changed_pcl = []
        self.lost_volume = None

        #PCL Collector
        self.mesh_folder_path = os.path.join(os.getcwd(), 'meshes_pair')
        if not os.path.exists(self.mesh_folder_path):
            os.mkdir(self.mesh_folder_path)
        self.mesh_filename = 'sample_plate'

        # Visualization threading
        self.lock = threading.Lock()
        self.visualization_thread = None


    def init_parameters(self) -> None:
        self.declare_parameter('dist_threshold',                '0.0006')    # filter for plane projection
        self.declare_parameter('plane_error_allowance',         '10'    )    # in degree
        self.declare_parameter('clusterscan_eps',               '0.0005')    # size for cluster grouping in DBScan
        self.declare_parameter('cluster_neighbor',              '30'    )    # number of required neighbour points to remove outliers
        self.declare_parameter('laserline_threshold',           '0.0001')    # scan resolution line axis in m
        self.declare_parameter('feedaxis_threshold',            '0.0002')    # scan resolution feed axis in m
        self.declare_parameter('plate_thickness',               '0.002' )    # in m

        self.dist_threshold = float(self.get_parameter('dist_threshold').get_parameter_value().string_value)
        self.cluster_neighbor = int(self.get_parameter('cluster_neighbor').get_parameter_value().string_value) 
        self.plate_thickness = float(self.get_parameter('plate_thickness').get_parameter_value().string_value) 
        self.plane_error_allowance = float(self.get_parameter('plane_error_allowance').get_parameter_value().string_value)
        self.clusterscan_eps = float(self.get_parameter('clusterscan_eps').get_parameter_value().string_value)
        self.laserline_threshold = float(self.get_parameter('laserline_threshold').get_parameter_value().string_value)
        self.feedaxis_threshold = float(self.get_parameter('feedaxis_threshold').get_parameter_value().string_value)

    def combined_cloud_callback(self, msg):
        self.get_logger().info('Received combined point cloud message')

        # Convert PointCloud2 message to Open3D point cloud
        o3d_pcl = self.convert_ros_to_open3d(msg)


        # if no initial scan
        if len(self.combined_pcl_initial) < 1:
            self.combined_pcl_initial.append(o3d_pcl)
            self.get_logger().info('PCL stored to initial')
            # visualize
            self.start_visualization_thread(self.combined_pcl_initial[0])

        elif len(self.combined_pcl_initial) > 0 and len(self.combined_pcl_postgrind) < 1:

            self.combined_pcl_postgrind.append(o3d_pcl)
            self.get_logger().info('PCL saved to postgrind')

            # TODO move this to data_gathering
            #Write PCL Pair to folder
            self.write_PCL_pair_to_folder()

            #filter point by plane and project onto it
            self.combined_pcl_initial[0], mesh1_plane_normal, mesh1_plane_centroid = filter_project_points_by_plane(self.combined_pcl_initial[0], distance_threshold=self.dist_threshold)
            self.combined_pcl_postgrind[0], mesh2_plane_normal, mesh2_plane_centroid = filter_project_points_by_plane(self.combined_pcl_postgrind[0], distance_threshold=self.dist_threshold)

            self.get_logger().info('PCL Projected on plane')

            # Check alignment
            cos_angle = np.dot(mesh1_plane_normal, mesh2_plane_normal) / (np.linalg.norm(mesh1_plane_normal) * np.linalg.norm(mesh2_plane_normal))
            angle = np.arccos(np.clip(cos_angle, -1.0, 1.0)) * 180 / np.pi
            if abs(angle) > self.plane_error_allowance:     #10 degree misalignment throw error
                raise ValueError(f"Plane normals differ too much: {angle} degrees")

            projected_points_mesh2 = project_points_onto_plane(np.asarray(self.combined_pcl_postgrind[0].points), mesh1_plane_normal, mesh1_plane_centroid)
            self.combined_pcl_postgrind[0].points = o3d.utility.Vector3dVector(projected_points_mesh2)


            mesh1_colored = self.combined_pcl_initial[0].paint_uniform_color([1, 0, 0])  # Red color
            mesh2_colored = self.combined_pcl_postgrind[0].paint_uniform_color([0, 1, 0])  # Green color

            self.get_logger().info('Filtering for changes in pcl')
            self.changed_pcl = filter_missing_points_by_yz(self.combined_pcl_initial[0], self.combined_pcl_postgrind[0], y_threshold=self.feedaxis_threshold, z_threshold=self.laserline_threshold)
            # after filter difference
            self.changed_pcl.paint_uniform_color([0, 0, 1])  # Blue color for changed surface mesh
            # after sorting
            self.changed_pcl = sort_largest_cluster(self.changed_pcl, eps=self.clusterscan_eps, min_points=self.cluster_neighbor, remove_outliers=True)

            self.start_visualization_thread(self.changed_pcl)
            #area from bounding box
            width, height, area = create_bbox_from_pcl(self.changed_pcl)

            self.get_logger().info(f"bbox width: {width} m, height: {height} m")
            self.lost_volume = area * self.plate_thickness
            self.get_logger().info(f"Lost Volume: {self.lost_volume} m^3")

            # Prepare and publish volume message
            msg_stamped = Float32Stamped()
            msg_stamped.data = float(self.lost_volume)
            self.publisher_volume.publish(msg_stamped)

            #After publish/save, set postgrind as initial and empty postgrind
            self.combined_pcl_initial = copy.deepcopy(self.combined_pcl_postgrind)
            self.combined_pcl_postgrind = []


    def write_PCL_pair_to_folder(self):

        rpm = 9000
        force = 5

        self.pair_folder_path = os.path.join(self.mesh_folder_path, f'rpm{rpm}_force{force}')
        if not os.path.exists(self.pair_folder_path):
            os.mkdir(self.pair_folder_path)

        # Write initial point cloud
        pcl_initial_path = os.path.join(self.pair_folder_path, f'pcl_{self.mesh_filename}_initial_{str(datetime.now()).split(".")[0]}.ply')
        self.get_logger().info(f"Saving initial pointcloud to: {pcl_initial_path}")
        o3d.io.write_point_cloud(pcl_initial_path, self.combined_pcl_initial[0])

        # Write postgrind point cloud
        pcl_postgrind_path = os.path.join(self.pair_folder_path, f'pcl_{self.mesh_filename}_postgrind_{str(datetime.now()).split(".")[0]}.ply')
        self.get_logger().info(f"Saving postgrind pointcloud to: {pcl_postgrind_path}")
        o3d.io.write_point_cloud(pcl_postgrind_path, self.combined_pcl_postgrind[0])

    def start_visualization_thread(self, pcl):
        # Avoid multiple visualization threads
        if self.visualization_thread is not None and self.visualization_thread.is_alive():
            return

        # Start the visualization thread
        self.visualization_thread = threading.Thread(target=self.visualize_mesh, args=(pcl,))
        self.visualization_thread.start()


    def visualize_mesh(self, mesh):
        # Initialize visualizer
        vis = o3d.visualization.VisualizerWithKeyCallback()
        vis.create_window(window_name="Mesh display", width=800, height=600)
        vis.add_geometry(mesh)

        def close_window(vis):
            vis.close()

        # Register a callback for 'ESC' key to close the window
        vis.register_key_callback(ord("Q"), close_window)  # 'Q' for quit or ESC
        vis.register_key_callback(256, close_window)  # ESC key for quitting

        # Main rendering loop
        while vis.poll_events():
            vis.update_renderer()

            # Check for shutdown flag from ROS or some other exit condition
            if not rclpy.ok():
                break

        # Destroy the window and clean up the visualizer
        vis.destroy_window()

    def convert_ros_to_open3d(self, pcl_msg):
        # Extract the point cloud data from the ROS2 message
        field_names = [field.name for field in pcl_msg.fields]
        pcl_data = list(point_cloud2.read_points(pcl_msg, skip_nans=True, field_names=field_names))

        # Convert to numpy array (only x, y, z coordinates)
        points = np.array([[p[0], p[1], p[2]] for p in pcl_data], dtype=np.float32)

        # Create an Open3D point cloud object
        o3d_pcl = o3d.geometry.PointCloud()
        o3d_pcl.points = o3d.utility.Vector3dVector(points)

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
        pcl_processor.visualization_thread.join()  # Ensure visualization thread completes
        pcl_processor.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()