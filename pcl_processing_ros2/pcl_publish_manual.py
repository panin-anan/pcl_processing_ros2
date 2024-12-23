import rclpy 
from rclpy.node import Node
from rclpy.impl import rcutils_logger
from rclpy.executors import MultiThreadedExecutor

from keyboard_msgs.msg import Key
from sensor_msgs.msg import PointCloud2, PointField

import open3d as o3d
import numpy as np
import tkinter as tk
from tkinter import filedialog, messagebox


from rcl_interfaces.msg import ParameterDescriptor 
from data_gathering_msgs.srv import RequestPCLVolumeDiff
from std_msgs.msg import Empty, String 
from sensor_msgs.msg import PointCloud2
from std_srvs.srv import Trigger 

import copy
import os 
import pathlib 
from datetime import datetime
import pandas as pd 
import numpy as np
import open3d as o3d
from functools import partial


class PCLpublisher(Node):
    def __init__(self):
        super().__init__('pcl_publish_manual')
        
        # Subscriber for the combined point cloud

        self.combine_pcl_publisher = self.create_publisher(PointCloud2,'combined_cloud',10)
        self.create_subscription(Key, 'keydown', self.key_callback, 1)

        self.calculate_volume_trigger = self.create_client(RequestPCLVolumeDiff, 'calculate_volume_lost')

        self.cloud_publish_trigger = Key.KEY_P
        self.global_frame_id = 'base_link'

        self.plate_thickness = 0.004  # Initialize plate thickness in m

        self.test_index = 0  # Track number of tests
        self.settings = []  # This should hold your test settings/requests
        self.initial_scan = None
        self.final_scan = None


    def key_callback(self, msg):
        if msg.code == self.cloud_publish_trigger:
            comb_cloud_pcl = self.load_mesh()
            self.get_logger().info('Loading PCL')

            if self.test_index == 0:
                self.initial_scan = self.create_pcl_msg(comb_cloud_pcl)
                self.combine_pcl_publisher.publish(self.create_pcl_msg(comb_cloud_pcl))
                self.get_logger().info('PCL loaded to initial')
                self.test_index += 1
            else:
                self.final_scan = self.create_pcl_msg(comb_cloud_pcl)
                self.combine_pcl_publisher.publish(self.create_pcl_msg(comb_cloud_pcl))
                self.get_logger().info('PCL loaded to final')
                #perform volume calculation
                req = RequestPCLVolumeDiff.Request()
                req.initial_pointcloud  = self.initial_scan
                req.final_pointcloud    = self.final_scan
                req.plate_thickness     = self.plate_thickness
                req.belt_width          = 0.025 #in m
                volume_call = self.calculate_volume_trigger.call_async(req)

                #prepare for next consecutive set of data
                self.initial_scan = self.final_scan
                self.test_index += 1

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

    def load_mesh(self):
        path = filedialog.askopenfilename(title=f"Select the mesh file for Mesh",
                                          filetypes=[("PLY files", "*.ply"), ("All Files", "*.*")])
        if path:
            try:
                # Try loading as a triangle mesh
                mesh = o3d.io.read_triangle_mesh(path)
                if len(mesh.triangles) > 0:
                    messagebox.showinfo("Success", f"Mesh Loaded Successfully (Triangle Mesh)")
                else:
                    # If no triangles, load as a point cloud
                    mesh = o3d.io.read_point_cloud(path)
                    if len(mesh.points) > 0:
                        messagebox.showinfo("Success", f"Mesh Loaded Successfully (Point Cloud)")
                    else:
                        messagebox.showwarning("Warning", f"File for Mesh contains no recognizable mesh or point cloud data.")

                return mesh

            except Exception as e:
                messagebox.showerror("Error", f"Failed to load Mesh: {str(e)}")
        else:
            messagebox.showwarning("Warning", f"No file selected for Mesh")

def main(args=None):
    rclpy.init(args=args)

    pcl_processor = PCLpublisher()
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