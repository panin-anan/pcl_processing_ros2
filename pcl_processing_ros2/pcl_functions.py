
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from sklearn.decomposition import PCA
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from concave_hull import concave_hull, concave_hull_indexes
import copy

class PCLfunctions:
    def __init__(self):
        self.ransac_iteration = 1000

    def fit_plane_to_pcd_pca(self, pcd):
            """Fit a plane to a cluster of points using PCA."""
            points = np.asarray(pcd.points)

            # Perform PCA
            pca = PCA(n_components=3)
            pca.fit(points)

            # Get the normal to the plane (third principal component)
            pca_basis = pca.components_  # Shape (3, 3)

            # The mean of the data gives the centroid
            centroid = pca.mean_  # Shape (3,)

            return pca_basis, centroid

    def project_points_onto_plane(self, points, plane_normal, plane_point):
        """Project points onto the plane defined by the normal and a point."""
        vectors = points - plane_point  # Vector from point to plane_point
        distances = np.dot(vectors, plane_normal)  # Project onto the normal
        projected_points = points - np.outer(distances, plane_normal)  # Subtract projection along the normal
        return projected_points

    def filter_project_points_by_plane(self, point_cloud, distance_threshold=0.001):
        # Fit a plane to the point cloud using RANSAC
        plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold,
                                                         ransac_n=3,
                                                         num_iterations=self.ransac_iteration)
        [a, b, c, d] = plane_model

        # Select points that are close to the plane (within the threshold)
        inlier_cloud = point_cloud.select_by_index(inliers)
        pca_basis, plane_centroid = self.fit_plane_to_pcd_pca(inlier_cloud)
        points = np.asarray(inlier_cloud.points)
        projected_points = self.project_points_onto_plane(points, pca_basis[2], plane_centroid)

        # Create a new point cloud with the projected points
        projected_pcd = o3d.geometry.PointCloud()
        projected_pcd.points = o3d.utility.Vector3dVector(projected_points)
        # projected_pcd.paint_uniform_color([1, 0, 0])  # Color projected points in red

        # Color the inlier points (on the original RANSAC plane) in green
        # inlier_cloud.paint_uniform_color([0, 1, 0])  # Color inlier points in green

        # Visualize both the original point cloud, inliers, and projected points
        #o3d.visualization.draw_geometries([projected_pcd])

        return projected_pcd, pca_basis, plane_centroid



    def filter_missing_points_by_xy(self, mesh_before, mesh_after, x_threshold=0.0003, y_threshold=0.0001):
        # Convert points from Open3D mesh to numpy arrays
        points_before = np.asarray(mesh_before.points)
        points_after = np.asarray(mesh_after.points)

        # Create a KDTree for the points in mesh_after
        kdtree_after = cKDTree(points_after)

        # Query KDTree to find distances and indices of nearest neighbors in mesh_after for points in mesh_before
        _, indices = kdtree_after.query(points_before)

        # Get the y and z coordinates from both meshes
        x_before = points_before[:, 0]
        y_before = points_before[:, 1]
        x_after = points_after[indices, 0]  # Nearest neighbors' y-coordinates
        y_after = points_after[indices, 1]  # Nearest neighbors' z-coordinates

        # Calculate the absolute differences in the y and z coordinates
        x_diff = np.abs(x_before - x_after)
        y_diff = np.abs(y_before - y_after)

        # Create a mask to find points in mesh_before where either the y or z axis difference
        # with the corresponding point in mesh_after exceeds the respective thresholds
        xy_diff_mask = (x_diff >= x_threshold) | (y_diff >= y_threshold)

        # Select the points from mesh_before where the y or z axis difference is larger than the threshold
        missing_points = points_before[xy_diff_mask]

        # Create a new point cloud with the points that have significant y or z axis differences
        mesh_missing = o3d.geometry.PointCloud()
        mesh_missing.points = o3d.utility.Vector3dVector(missing_points)

        return mesh_missing


    def create_bbox_from_pcl(self, pcl):
        # Step 1: Convert point cloud to numpy array
        points = np.asarray(pcl.points)

        # Step 2: Since data is planar, project points to a 2D plane (ignore one axis, e.g., Z-axis)
        xy_points = points[:, 0:2]  # Take X and Y coordinates (planar in XY plane)

        # Step 3: Get the 2D Axis-Aligned Bounding Box (AABB) for the planar points (XY plane)
        min_bound = np.min(xy_points, axis=0)
        max_bound = np.max(xy_points, axis=0)

        # Step 4: Calculate glob_z and glob_y of the 2D bounding box (in XY plane)
        glob_z = max_bound[0] - min_bound[0]  # X-axis difference
        glob_y = max_bound[1] - min_bound[1]  # Y-axis difference

        # Step 5: Calculate the 2D area (in XY plane)
        area = glob_z * glob_y

        return glob_z, glob_y, area

    def create_bbox_from_pcl_axis_aligned(self, pcl):
        # Step 1: Convert point cloud to numpy array
        points = np.asarray(pcl.points)
        dummy_pcl = copy.deepcopy(pcl)
        # Step 2: Initialize variables to store the minimum glob_z and corresponding bounding box
        min_glob_z = float('inf')
        min_glob_y = float('inf')
        best_bbox = None
        best_axes = None
        best_angle = 0
        
        # Step 3: Iterate over angles to find the orientation with minimal bounding box glob_z
        for angle in np.linspace(0, np.pi, 100):  # 100 steps from 0 to 180 degrees
            # Step 4: Create the 2D rotation matrix around the Z-axis
            rotation_matrix = np.array([
                [np.cos(angle), -np.sin(angle)],
                [np.sin(angle), np.cos(angle)]
            ])
            
            # Step 5: Rotate points around the Z-axis in the XY plane
            xy_points = points[:, 0:2]  # Take X and Y coordinates (planar in XY plane)
            rotated_points = np.dot(xy_points, rotation_matrix.T)
            
            # Step 6: Get the Axis-Aligned Bounding Box (AABB) for the rotated points
            min_bound = np.min(rotated_points, axis=0)
            max_bound = np.max(rotated_points, axis=0)
            
            # Calculate glob_z and glob_y of the bounding box
            glob_z = max_bound[0] - min_bound[0]
            glob_y = max_bound[1] - min_bound[1]
            
            # Step 7: If this rotation gives a smaller glob_z, update the minimum glob_z and bbox
            if glob_z < min_glob_z:
                min_glob_z = glob_z
                min_glob_y = glob_y
                min_bound_3d = np.append(min_bound, 0)  # Append Z=0
                max_bound_3d = np.append(max_bound, 0)  # Append Z=0
                bbox = o3d.geometry.AxisAlignedBoundingBox(min_bound=min_bound_3d, max_bound=max_bound_3d)
                best_bbox = bbox
                best_angle = angle
    
        final_rotation_matrix = np.array([
            [np.cos(best_angle), -np.sin(best_angle), 0],
            [np.sin(best_angle), np.cos(best_angle), 0],
            [0, 0, 1]
        ])

        area = min_glob_z * min_glob_y

        return min_glob_z, min_glob_y, area

    def compute_convex_hull_area_xy(self, point_cloud):
        # Step 1: Convert point cloud to numpy array
        points = np.asarray(point_cloud.points)

        # Step 2: Project the points onto the XY plane
        xy_points = points[:, 0:2]  # Extract only the X and Y coordinates

        # Step 3: Compute the convex hull using scipy's ConvexHull on the projected XY points
        hull_2d = ConvexHull(xy_points)

        # Step 4: The area of the convex hull (in the XY plane)
        area = hull_2d.volume

        return area, hull_2d

    def compute_concave_hull_area_xy(self, point_cloud, hull_convex_2d, concave_resolution=0.0005):
        points = np.asarray(point_cloud.points)

        #plt.scatter(points[:, 0], points[:, 1], s=0.5, color='b', alpha=0.5)

        idxes = concave_hull_indexes(
            points[:, :2],
            length_threshold=concave_resolution,
        )
        # you can get coordinates by `points[idxes]`
        assert np.all(points[idxes] == concave_hull(points, length_threshold=concave_resolution))

        for f, t in zip(idxes[:-1], idxes[1:]):  # noqa
            seg = points[[f, t]]
            #plt.plot(seg[:, 0], seg[:, 1], "r-", alpha=0.5)
        # plt.savefig('hull.png')
        #plt.gca().set_aspect('equal', adjustable='box')
        #plt.show()

        # Calculate the area using the Shoelace formula
        hull_points = points[idxes]
        x = hull_points[:, 0]
        y = hull_points[:, 1]
        area = 0.5 * np.abs(np.dot(x, np.roll(y, 1)) - np.dot(y, np.roll(x, 1)))

        hull_cloud = o3d.geometry.PointCloud()
        hull_cloud.points = o3d.utility.Vector3dVector(hull_points)

        return area, hull_cloud

    def sort_plate_cluster(self, pcd, eps=0.0005, min_points=30, remove_outliers=False, use_downsampling=False, downsample_voxel_size=0.0002):
        if use_downsampling and downsample_voxel_size > 0:
            downsampled_pcd = pcd.voxel_down_sample(voxel_size=downsample_voxel_size)
        else:
            downsampled_pcd = pcd

        # Step 2: Segment downsampled point cloud into clusters using DBSCAN
        labels = np.array(downsampled_pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

        # Number of clusters (label -1 indicates noise)
        num_clusters = labels.max() + 1
        if num_clusters == 0:
            return o3d.geometry.PointCloud()  # Return empty point cloud if no clusters are found

        # Step 3: Find the largest cluster in the downsampled point cloud
        max_cluster_size = 0
        largest_cluster_indices = None

        for cluster_idx in range(num_clusters):
            # Get the indices of the points that belong to the current cluster
            cluster_indices = np.where(labels == cluster_idx)[0]
            # If this cluster is the largest we've found, update the largest cluster info
            if len(cluster_indices) > max_cluster_size:
                max_cluster_size = len(cluster_indices)
                largest_cluster_indices = cluster_indices

        if largest_cluster_indices is None:
            return o3d.geometry.PointCloud()  # Return empty point cloud if no largest cluster is found

        # Step 4: Map the largest cluster back to the original point cloud
        largest_cluster_pcd = downsampled_pcd.select_by_index(largest_cluster_indices)

        # Find corresponding points in the original high-resolution point cloud
        distances = pcd.compute_point_cloud_distance(largest_cluster_pcd)
        original_cluster_indices = np.where(np.asarray(distances) < downsample_voxel_size*10)[0]  # Tolerance to find nearest neighbors
        high_res_largest_cluster_pcd = pcd.select_by_index(original_cluster_indices)

        # Optionally remove outliers from the largest cluster
        #if remove_outliers and high_res_largest_cluster_pcd is not None:
        #    high_res_largest_cluster_pcd, _ = high_res_largest_cluster_pcd.remove_radius_outlier(nb_neighbors=5, std_ratio=)

        return high_res_largest_cluster_pcd

    def sort_largest_cluster(self, pcd, eps=0.005, min_points=30, remove_outliers=True, outlier_eps=0.0002):
        # Step 1: Segment point cloud into clusters using DBSCAN
        labels = np.array(pcd.cluster_dbscan(eps=eps, min_points=min_points, print_progress=True))

        # Number of clusters (label -1 indicates noise)
        num_clusters = labels.max() + 1

        # colors = plt.get_cmap("tab20")(labels / (num_clusters if num_clusters > 0 else 1))
        # colors[labels == -1] = [0, 0, 0, 1]  # Color noise points black
        # pcd.colors = o3d.utility.Vector3dVector(colors[:, :3])
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
        if remove_outliers and largest_cluster_pcd is not None:
            before_outrem = len(largest_cluster_pcd.points)
            largest_cluster_pcd, _ = largest_cluster_pcd.remove_radius_outlier(nb_points=5, radius=outlier_eps*1.5)
            after_outrem = len(largest_cluster_pcd.points)
            outliers_removed = before_outrem - after_outrem

        if largest_cluster_pcd is None:
            largest_cluster_pcd = o3d.geometry.PointCloud()

        return largest_cluster_pcd, outliers_removed

    def transform_to_local_pca_coordinates(self, pcd, pca_basis, centroid):
        points = np.asarray(pcd.points)
        centered_points = points - centroid
        local_points = centered_points @ pca_basis.T
        local_pcl = o3d.geometry.PointCloud()
        local_pcl.points = o3d.utility.Vector3dVector(local_points)
        return local_pcl

    def transform_to_global_coordinates(self, local_pcl, pca_basis, centroid):
        local_points = np.asarray(local_pcl.points)

        # Step 1: Apply the inverse PCA transformation (PCA basis transpose, since it's orthonormal)
        global_points = local_points @ pca_basis

        # Step 2: Add the centroid back to translate the points back to the original coordinate system
        global_points += centroid

        # Step 3: Create a new PointCloud with global coordinates
        global_pcl = o3d.geometry.PointCloud()
        global_pcl.points = o3d.utility.Vector3dVector(global_points)

        return global_pcl

    def filter_points_by_plane_nearbycloud(self, point_cloud, distance_threshold=0.0008, nearby_threshold=0.01):
        # Fit a plane to the point cloud using RANSAC
        plane_model, inliers = point_cloud.segment_plane(distance_threshold=distance_threshold,
                                                         ransac_n=3,
                                                         num_iterations=1000)
        [a, b, c, d] = plane_model

        # Select points that are close to the plane (within the threshold)
        inlier_cloud = point_cloud.select_by_index(inliers)
        pca_basis, plane_centroid = self.fit_plane_to_pcd_pca(inlier_cloud)
        points = np.asarray(inlier_cloud.points)

        # Select additional points within the specified nearby distance to the plane
        distances = np.abs(np.dot(np.asarray(point_cloud.points), [a, b, c]) + d) / np.linalg.norm([a, b, c])
        nearby_indices = np.where(distances <= nearby_threshold)[0]
        nearby_cloud = point_cloud.select_by_index(nearby_indices)

        # Color the inlier points (on the original RANSAC plane) in green
        inlier_cloud.paint_uniform_color([0, 1, 0])  # Color inlier points in green
        nearby_cloud.paint_uniform_color([0, 0, 1])  # Color nearby points in blue

        # Visualize both the inliers, projected points, and nearby points
        #o3d.visualization.draw_geometries([projected_pcd, inlier_cloud, nearby_cloud])

        return nearby_cloud, pca_basis, plane_centroid

    def shift_y_fordiagonal(self, pcd):
        # Extract points
        points = np.asarray(pcd.points)

        # Define y range for applying the shift
        min_y = 0.816
        max_y = min_y + 0.009
        max_shift = 0.002

        # Calculate shift factor for points within the specified y range
        mask = (points[:, 1] > min_y) & (points[:, 1] <= max_y)
        points[mask, 0] += max_shift * (points[mask, 1] - min_y) / (max_y - min_y)

        # Update point cloud with modified points
        pcd.points = o3d.utility.Vector3dVector(points)

        return pcd

    def filter_changedpoints_onNormZaxis(self, mesh_before, mesh_after, z_threshold=0.0003, x_threshold=0.0001, z_threshold_after=0.0001, neighbor_threshold=5):
        # Convert points from Open3D mesh to numpy arrays
        points_before = np.asarray(mesh_before.points)
        points_after = np.asarray(mesh_after.points)

        # Create a KDTree for the points in mesh_after
        kdtree_after = cKDTree(points_after)

        # Query KDTree to find distances and indices of nearest neighbors in mesh_after for points in mesh_before
        _, indices = kdtree_after.query(points_before)

        # Get the x and z coordinates from both meshes
        z_before = points_before[:, 2]
        x_before = points_before[:, 0]
        z_after = points_after[indices, 2]  # Nearest neighbors' z-coordinates
        x_after = points_after[indices, 0]  # Nearest neighbors' x-coordinates

        # Calculate the absolute differences in the x and z coordinates
        z_diff = np.abs(z_before - z_after)
        x_diff = np.abs(x_before - x_after)

        # Create a mask to find points in mesh_before where either the x or z axis difference
        # with the corresponding point in mesh_after exceeds the respective thresholds
        zx_diff_mask = (z_diff >= z_threshold) | (x_diff >= x_threshold)

        # Select the points from mesh_before where the x or z axis difference is larger than the threshold
        missing_points = points_before[zx_diff_mask]

        # Create a new point cloud with the points that have significant x or z axis differences
        mesh_missing = o3d.geometry.PointCloud()
        mesh_missing.points = o3d.utility.Vector3dVector(missing_points)

        # Now find points in mesh_after that are outside the z_threshold distance from mesh_missing
        kdtree_missing_z = cKDTree(missing_points[:, [2]])  # KDTree with only z-coordinates
        distances, _ = kdtree_missing_z.query(points_after[:, [2]])

        # Mask to filter points in mesh_after that are outside the z_threshold distance in z-axis
        change_mask = distances > z_threshold_after
        changed_points = points_after[change_mask]

        # Create mesh_change with points in mesh_after that are outside the z-axis threshold distance from mesh_missing
        mesh_change = o3d.geometry.PointCloud()
        mesh_change.points = o3d.utility.Vector3dVector(changed_points)

        return mesh_missing, mesh_change

    def calculate_volume_with_projected_boundaries_concave(self, pcd1, pcd2, num_slices=3, concave_resolution=0.002):
        """
        Calculate the volume between two irregular surfaces by integrating cross-sectional areas along the y-axis.
        """
        points1 = np.asarray(pcd1.points)
        points2 = np.asarray(pcd2.points)

        # Define full x, y, and z bounds based on the two surfaces
        x_min = min(points1[:, 0].min(), points2[:, 0].min())
        x_max = max(points1[:, 0].max(), points2[:, 0].max())
        y_min = min(points1[:, 1].min(), points2[:, 1].min())
        y_max = max(points1[:, 1].max(), points2[:, 1].max())
        z_min = min(points1[:, 2].min(), points2[:, 2].min())
        z_max = max(points1[:, 2].max(), points2[:, 2].max())

        # Calculate the area of the xz-plane bounding box
        bounding_area_xz = (x_max - x_min) * (z_max - z_min)

        # Define y-axis slices
        y_slices = np.linspace(y_min, y_max, num_slices)
        total_volume = 0
        hull_clouds = []

        for i in range(len(y_slices) - 1):
            y_start, y_end = y_slices[i], y_slices[i + 1]
            slice_thickness = y_end - y_start
            y_mid = (y_start + y_end) / 2

            # Filter points within the current y-slice for each surface
            slice_points1 = points1[(points1[:, 1] >= y_start) & (points1[:, 1] < y_end)]
            slice_points2 = points2[(points2[:, 1] >= y_start) & (points2[:, 1] < y_end)]

            # Combine points from both surfaces for this slice
            combined_slice_points = np.vstack((slice_points1, slice_points2))

            if len(combined_slice_points) >= 3:
                # Project combined points onto the xz-plane at y_mid
                xz_combined_points = np.column_stack((combined_slice_points[:, 0], np.full(combined_slice_points.shape[0], y_mid), combined_slice_points[:, 2]))
                xz_points_display = o3d.geometry.PointCloud()
                xz_points_display.points = o3d.utility.Vector3dVector(xz_combined_points)


                # Compute the concave hull indices
                idxes = concave_hull_indexes(xz_combined_points[:, [0, 2]], length_threshold=concave_resolution)
                hull_points = xz_combined_points[idxes]

                # Calculate the area of the concave hull using the Shoelace formula
                x = hull_points[:, 0]
                z = hull_points[:, 2]
                cross_sectional_area = 0.5 * np.abs(np.dot(x, np.roll(z, 1)) - np.dot(z, np.roll(x, 1)))

                # Prepare visualization of Concave Hull in Open3D
                hull_cloud = o3d.geometry.PointCloud()
                hull_cloud.points = o3d.utility.Vector3dVector(hull_points)
                hull_clouds.append(hull_cloud)

                # Create lines to connect hull points in sequence and close the loop
                hull_lines = [[j, (j + 1) % len(idxes)] for j in range(len(idxes))]

                hull_line_set = o3d.geometry.LineSet()
                hull_line_set.points = hull_cloud.points
                hull_line_set.lines = o3d.utility.Vector2iVector(hull_lines)

                # Display the concave hull with Open3D
                #o3d.visualization.draw_geometries([xz_points_display, hull_line_set], window_name=f"Slice {i+1} (y_mid = {y_mid:.2f})")

            else:
                # Use the bounding area if there are insufficient points to form a polygon
                cross_sectional_area = bounding_area_xz

            # Estimate volume for this slice
            slice_volume = cross_sectional_area * slice_thickness

            # Add the slice volume to the total volume
            total_volume += slice_volume

        # Return the total volume
        return total_volume, hull_clouds