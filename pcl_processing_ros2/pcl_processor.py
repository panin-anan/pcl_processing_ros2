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
import copy
from scipy.spatial import cKDTree


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

        # Visualization threading
        self.lock = threading.Lock()
        self.visualization_thread = None


    def init_parameters(self) -> None:
        self.declare_parameter('distance_filter_threshold',     '0.0005')    # filter for segmenting grinded area
        self.declare_parameter('neighbor_threshold',            '5'     )    # number of required neighbour points to remove outliers
        self.declare_parameter('plate_thickness',               '2'     )    # in mm

        self.distance_filter_threshold = float(self.get_parameter('distance_filter_threshold').get_parameter_value().string_value)
        self.neighbor_threshold = float(self.get_parameter('neighbor_threshold').get_parameter_value().string_value) 
        self.plate_thickness = float(self.get_parameter('plate_thickness').get_parameter_value().string_value) 

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
            self.get_logger().info('Processing postgrind')
            #Filter for grinded area and visualize
            self.changed_pcl = self.filter_changedpointson_mesh(self.combined_pcl_initial[0], self.combined_pcl_postgrind[0])

            #self.start_visualization_thread(self.changed_pcl)

            # Create mesh and calculate volume
            changed_mesh_surf = self.create_mesh_from_point_cloud(self.changed_pcl)

            self.start_visualization_thread(changed_mesh_surf)

            self.lost_volume = self.calculate_lost_volume_from_changedpcl(changed_mesh_surf)  
            self.get_logger().info(f"Lost Volume: {self.lost_volume} cubic mm")

            # Prepare and publish volume message
            msg_stamped = Float32Stamped()
            msg_stamped.data = float(self.lost_volume)
            msg_stamped.header = Header()
            self.publisher_volume.publish(msg_stamped)

            #After publish, set postgrind as initial and empty postgrind
            self.combined_pcl_initial = copy.deepcopy(self.combined_pcl_postgrind)
            self.combined_pcl_postgrind = []


    def start_visualization_thread(self, pcl):
        # Avoid multiple visualization threads
        if self.visualization_thread is not None and self.visualization_thread.is_alive():
            return

        # Start the visualization thread
        self.visualization_thread = threading.Thread(target=self.visualize_mesh, args=(pcl,))
        self.visualization_thread.start()

    def filter_changedpointson_mesh(self, mesh_before, mesh_after):
        points_before = np.asarray(mesh_before.points)
        points_after = np.asarray(mesh_after.points)

        # Create KDTree for the points
        kdtree_after = cKDTree(points_after)
        distances, indices = kdtree_after.query(points_before)

        # Filter points in mesh_before that are not within the self.distance_filter_threshold distance in mesh_after
        missing_indices = np.where(distances >= self.distance_filter_threshold)[0]
        missing_vertices = points_before[missing_indices]

        # Create a new point cloud with points that are missing in mesh_after
        mesh_missing = o3d.geometry.PointCloud()
        mesh_missing.points = o3d.utility.Vector3dVector(missing_vertices)

        # Now, filter out points in mesh_missing that have fewer than 20 neighbors within the self.distance_filter_threshold distance
        missing_vertices_np = np.asarray(mesh_missing.points)

        # Create KDTree for the points in mesh_missing
        kdtree_missing = cKDTree(missing_vertices_np)

        # Query neighbors within the self.distance_filter_threshold distance for each point in mesh_missing
        neighbor_counts = kdtree_missing.query_ball_point(missing_vertices_np, r=self.distance_filter_threshold)

        # Only keep points that have at least 20 neighbors in their vicinity
        valid_indices = [i for i, neighbors in enumerate(neighbor_counts) if len(neighbors) >= self.neighbor_threshold]
        valid_vertices = missing_vertices_np[valid_indices]

        # Create a new point cloud with the filtered points
        filtered_mesh_missing = o3d.geometry.PointCloud()
        filtered_mesh_missing.points = o3d.utility.Vector3dVector(valid_vertices)

        return filtered_mesh_missing

    def calculate_lost_volume_from_changedpcl(self, mesh_missing):
        reference_area = mesh_missing.get_surface_area()
        volume_lost = self.plate_thickness * reference_area

        return volume_lost

    def create_mesh_from_point_cloud(self, pcd):
        points = np.asarray(pcd.points)
        jitter = np.random.normal(scale=1e-8, size=points.shape)
        pcd.points = o3d.utility.Vector3dVector(points + jitter)
        pcd.estimate_normals()

        pcd.orient_normals_consistent_tangent_plane(30)

        distances = pcd.compute_nearest_neighbor_distance()
        avg_dist = np.mean(distances)
        radii = [0.05 * avg_dist, 0.1 * avg_dist, 0.25 * avg_dist, 0.4 * avg_dist, 0.7 * avg_dist, 1 * avg_dist, 1.5 * avg_dist, 2 * avg_dist, 3 * avg_dist] #can reduce to reduce computation
        r = o3d.utility.DoubleVector(radii)

        #ball pivoting
        mesh = o3d.geometry.TriangleMesh.create_from_point_cloud_ball_pivoting(pcd, r)

        return mesh


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