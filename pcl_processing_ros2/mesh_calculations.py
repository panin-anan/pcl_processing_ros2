# mesh_calculations.py

import numpy as np
import open3d as o3d
import tkinter as tk
from tkinter import filedialog, messagebox
from scipy.spatial import cKDTree, Delaunay
import time
from scipy.spatial import ConvexHull
from sklearn.decomposition import PCA
import matplotlib.pyplot as plt

def fit_plane_to_pcd_pca(pcd):
        """Fit a plane to a cluster of points using PCA."""
        points = np.asarray(pcd.points)

        # Perform PCA
        pca = PCA(n_components=3)
        pca.fit(points)

        # Get the normal to the plane (third principal component)
        plane_normal = pca.components_[2]  # The normal to the plane (least variance direction)

        # The centroid is the mean of the points
        centroid = np.mean(points, axis=0)

        return plane_normal, centroid

def project_points_onto_plane(points, plane_normal, plane_point):
    """Project points onto the plane defined by the normal and a point."""
    vectors = points - plane_point  # Vector from point to plane_point
    distances = np.dot(vectors, plane_normal)  # Project onto the normal
    projected_points = points - np.outer(distances, plane_normal)  # Subtract projection along the normal
    return projected_points

def filter_project_points_by_plane(point_cloud, distance_threshold=0.001):
    # Fit a plane to the point cloud using RANSAC
    plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold,
                                                     ransac_n=3,
                                                     num_iterations=1000)
    [a, b, c, d] = plane_model
    
    # Select points that are close to the plane (within the threshold)
    inlier_cloud = point_cloud.select_by_index(inliers)
    
    plane_normal, plane_centroid = fit_plane_to_pcd_pca(inlier_cloud)
    points = np.asarray(inlier_cloud.points)
    projected_points = project_points_onto_plane(points, plane_normal, plane_centroid)

    # Create a new point cloud with the projected points
    projected_pcd = o3d.geometry.PointCloud()
    projected_pcd.points = o3d.utility.Vector3dVector(projected_points)
    projected_pcd.paint_uniform_color([1, 0, 0])  # Color projected points in red
    
    # Color the inlier points (on the original RANSAC plane) in green
    inlier_cloud.paint_uniform_color([0, 1, 0])  # Color inlier points in green

    # Visualize both the original point cloud, inliers, and projected points
    #o3d.visualization.draw_geometries([projected_pcd])

    return projected_pcd, plane_normal, plane_centroid



def filter_missing_points_by_yz(mesh_before, mesh_after, y_threshold=0.0003, z_threshold=0.0001):
    # Convert points from Open3D mesh to numpy arrays
    points_before = np.asarray(mesh_before.points)
    points_after = np.asarray(mesh_after.points)
    
    # Create a KDTree for the points in mesh_after
    kdtree_after = cKDTree(points_after)
    
    # Query KDTree to find distances and indices of nearest neighbors in mesh_after for points in mesh_before
    _, indices = kdtree_after.query(points_before)
    
    # Get the y and z coordinates from both meshes
    y_before = points_before[:, 1]
    z_before = points_before[:, 2]
    y_after = points_after[indices, 1]  # Nearest neighbors' y-coordinates
    z_after = points_after[indices, 2]  # Nearest neighbors' z-coordinates
    
    # Calculate the absolute differences in the y and z coordinates
    y_diff = np.abs(y_before - y_after)
    z_diff = np.abs(z_before - z_after)
    
    # Create a mask to find points in mesh_before where either the y or z axis difference
    # with the corresponding point in mesh_after exceeds the respective thresholds
    yz_diff_mask = (y_diff >= y_threshold) | (z_diff >= z_threshold)
    
    # Select the points from mesh_before where the y or z axis difference is larger than the threshold
    missing_points = points_before[yz_diff_mask]
    
    # Create a new point cloud with the points that have significant y or z axis differences
    mesh_missing = o3d.geometry.PointCloud()
    mesh_missing.points = o3d.utility.Vector3dVector(missing_points)
    
    return mesh_missing


def create_bbox_from_pcl(pcl):
    # Step 1: Convert point cloud to numpy array
    points = np.asarray(pcl.points)

    # Step 2: Since data is planar, project points to a 2D plane (ignore one axis, e.g., X-axis)
    yz_points = points[:, 1:3]  # Take Y and Z coordinates (planar in YZ plane)

    # Step 3: Get the 2D Axis-Aligned Bounding Box (AABB) for the planar points (YZ plane)
    min_bound = np.min(yz_points, axis=0)
    max_bound = np.max(yz_points, axis=0)

    # Step 4: Calculate width and height of the 2D bounding box (in YZ plane)
    width = max_bound[0] - min_bound[0]  # Y-axis difference
    height = max_bound[1] - min_bound[1]  # Z-axis difference

    # Step 5: Calculate the 2D area (in YZ plane)
    area = width * height

    return width, height, area


def sort_largest_cluster(pcd, eps=0.005, min_points=30, remove_outliers=True):
    # Step 1: Segment point cloud into clusters using DBSCAN
    labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))
    
    # Number of clusters (label -1 indicates noise)
    num_clusters = labels.max() + 1

    colors = plt.get_cmap("tab20")(labels / (num_clusters if num_clusters > 0 else 1))
    colors[labels == -1] = [0, 0, 0, 1]  # Color noise points black
    pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
    #o3d.visualization.draw_geometries([pcd])

    # Step 2: Find the largest cluster
    max_cluster_size = 0
    largest_cluster_pcd = None

    for cluster_idx in range(num_clusters):
        # Get the indices of the points that belong to the current cluster
        cluster_indices = np.where(labels == cluster_idx)[0]
        
        # If this cluster is the largest we've found, update the largest cluster info
        if len(cluster_indices) > max_cluster_size:
            max_cluster_size = len(cluster_indices)
            largest_cluster_pcd = pcd.select_by_index(cluster_indices)

    # Optionally: Remove outliers (if remove_outliers is set to True)
    #if remove_outliers and largest_cluster_pcd is not None:
    #    largest_cluster_pcd, _ = largest_cluster_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

    return largest_cluster_pcd