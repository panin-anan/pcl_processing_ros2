import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import Header, Empty
from sensor_msgs.msg import PointCloud2, PointField
from keyboard_msgs.msg import Key
from stamped_std_msgs.msg import Float32Stamped, Int32Stamped, TimeSync

import open3d as o3d
import numpy as np
import sensor_msgs_py.point_cloud2 as pc2

import copy 
from datetime import datetime
import os 


class PCLprocessor(Node):
    def __init__(self):
        super().__init__('pcl_processor')
        
        # Subscriber for the combined point cloud
        self.combinepcl_subscription = self.create_subscription(PointCloud2,'combined_cloud',self.combined_cloud_callback,10)

        #Publisher to publish volume lost
        self.publisher_volume = self.create_publisher(Float32Stamped, '/scanner/volume', 10)

        # Pointcloud storage
        self.combined_pcl_initial = []
        self.combined_pcl_postgrind = []

    def combined_cloud_callback(self, msg):
        self.get_logger().info('Received combined point cloud message')

        # Convert PointCloud2 message to Open3D point cloud
        o3d_pcl = self.convert_ros_to_open3d(msg)

        # Visualize the combined point cloud
        #o3d.visualization.draw_geometries([o3d_pcl], window_name="Received Combined Point Cloud")

        # if no initial scan
        if self.combined_pcl_initial:
            self.combined_pcl_initial.append(o3d_pcl)
            self.get_logger().info('PCL stored to initial')
        elif self.combined_pcl_initial and self.combined_pcl_postgrind:

            self.combined_pcl_postgrind = o3d_pcl

            #calculate volume
            lost_volume = calculate_lost_volume_from_changedpcl(self.combined_pcl_initial, self.combined_pcl_postgrind)
            msg_stamped.data = float(lost_volume)

            #publish volume.
            self.publisher_volume.publish(Float32Stamped(data=msg_stamped.data, header=Header()))

            #After publish, set postgrind as initial
            self.combined_pcl_initial = self.combined_pcl_postgrind

    def convert_ros_to_open3d(self, pcl_msg):
        o3d_combined_pcl = o3d.geometry.PointCloud()
        loaded_array = np.frombuffer(pcl_msg.data, dtype=np.float32).reshape(-1, 4) 
        
        loaded_array = loaded_array[np.where(loaded_array[:,2] > 1e-3)]
        o3d_pcl = o3d.geometry.PointCloud()
        o3d_pcl.points = o3d.utility.Vector3dVector(loaded_array[:,:3])
        return o3d_pcl

def main(args=None):
    rclpy.init(args=args)

    pcl_processor = PCLprocessor()
    executor = MultiThreadedExecutor()

    rclpy.spin(pcl_processor, executor=executor)
    pcl_processor.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()