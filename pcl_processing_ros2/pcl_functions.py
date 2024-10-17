
import numpy as np
import open3d as o3d
from scipy.spatial import cKDTree
from sklearn.decomposition import PCA
from scipy.spatial import ConvexHull
import matplotlib.pyplot as plt
from concave_hull import concave_hull, concave_hull_indexes

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

        # Step 4: Calculate width and height of the 2D bounding box (in XY plane)
        width = max_bound[0] - min_bound[0]  # X-axis difference
        height = max_bound[1] - min_bound[1]  # Y-axis difference

        # Step 5: Calculate the 2D area (in XY plane)
        area = width * height

        return width, height, area

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
        if remove_outliers and high_res_largest_cluster_pcd is not None:
            high_res_largest_cluster_pcd, _ = high_res_largest_cluster_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        return high_res_largest_cluster_pcd

    def sort_largest_cluster(self, pcd, eps=0.005, min_points=30, remove_outliers=True):
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
        #if remove_outliers and largest_cluster_pcd is not None:
        #    largest_cluster_pcd, _ = largest_cluster_pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)

        if largest_cluster_pcd is None:
            largest_cluster_pcd = o3d.geometry.PointCloud()

        return largest_cluster_pcd

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
    
    def calculate_volume_difference(self, pcl1, pcl2, plate_thickness, settings: dict, logger=None):
        #filter point by plane and project onto it
        pcl1, mesh1_pca_basis, mesh1_plane_centroid = self.filter_project_points_by_plane(pcl1, distance_threshold=settings['dist_threshold'])
        pcl2, mesh2_pca_basis, mesh2_plane_centroid = self.filter_project_points_by_plane(pcl2, distance_threshold=settings['dist_threshold'])
        pcl1_plane = pcl1
        pcl2_plane = pcl2
        pcl1 = self.sort_plate_cluster(pcl1_plane, eps=0.001, min_points=30, use_downsampling=True, downsample_voxel_size=settings['filter_down_size'])
        pcl2 = self.sort_plate_cluster(pcl2_plane, eps=0.001, min_points=30, use_downsampling=True, downsample_voxel_size=settings['filter_down_size'])

        # Check if the largest cluster has at least half the points of the original point cloud
        if len(pcl1.points) < len(pcl1_plane.points) / 2:
            if logger is not None:
                logger.info(f"voxel down algorithm failed. Resorting with original pcl")
            pcl1 = self.sort_plate_cluster(pcl1_plane, eps=0.0005, min_points=30, use_downsampling=False)
            pcl2 = self.sort_plate_cluster(pcl2_plane, eps=0.0005, min_points=30, use_downsampling=False)

        if logger is not None:
            logger.info('PCL Projected on plane')

        # Check alignment
        cos_angle = np.dot(mesh1_pca_basis[2], mesh2_pca_basis[2]) / (np.linalg.norm(mesh1_pca_basis[2]) * np.linalg.norm(mesh2_pca_basis[2]))
        angle = np.arccos(np.clip(cos_angle, -1.0, 1.0)) * 180 / np.pi
        if abs(angle) > settings['plane_error_allowance']:     #10 degree misalignment throw error
            if logger is not None:
                logger.error(f"Plane normals differ too much: {angle} degrees. Something may have moved in between recording the two pointclouds, or the wrong plane was detected for one or more of the pointclouds.")
            return None

        projected_points_mesh2 = self.project_points_onto_plane(np.asarray(pcl2.points), mesh1_pca_basis[2], mesh1_plane_centroid)
        pcl2.points = o3d.utility.Vector3dVector(projected_points_mesh2)

        #transform points to local xy plane
        pcl1_local = self.transform_to_local_pca_coordinates(pcl1, mesh1_pca_basis, mesh1_plane_centroid )
        pcl2_local = self.transform_to_local_pca_coordinates(pcl2, mesh1_pca_basis, mesh1_plane_centroid )

        if logger is not None:
            logger.info('Filtering for changes in pcl')
        changed_pcl_local = self.filter_missing_points_by_xy(pcl1_local, pcl2_local, x_threshold=settings['feedaxis_threshold'], y_threshold=settings['laserline_threshold'])
        changed_pcl_local_all = changed_pcl_local
        # after sorting
        changed_pcl_local = self.sort_largest_cluster(changed_pcl_local, eps=settings['clusterscan_eps'], min_points=settings['cluster_neighbor'], remove_outliers=True)

        hull_cloud_global = None 
        # Check if there are any missing points detected
        if changed_pcl_local is None or len(np.asarray(changed_pcl_local.points)) == 0:
            if logger is not None:
                logger.info("No detectable difference between point clouds. Lost volume is 0.")
            lost_volume = 0.0
        else: 
            #area from bounding box
            height, width, area_bb = self.create_bbox_from_pcl(changed_pcl_local)
            if width < settings['belt_width_threshold']:
                if logger is not None:
                    logger.info("clustering not appropriate. increasing threshold")
                changed_pcl_local = self.sort_largest_cluster(changed_pcl_local_all, eps=settings['clusterscan_eps']*5, min_points=settings['cluster_neighbor'], remove_outliers=True)
            #area from convex hull
            area, hull_convex_2d = self.compute_convex_hull_area_xy(changed_pcl_local)

            area_concave, hull_concave_2d_cloud = self.compute_concave_hull_area_xy(changed_pcl_local, hull_convex_2d, concave_resolution= settings['concave_resolution'])
            if logger is not None:
                logger.info(f"bbox width: {width * 1000} mm, height: {height * 1000} mm")
                logger.info(f"bbox area: {area_bb * (1000**2)} mm^2, concave_hull_area: {area_concave * (1000**2)} mm^2")
            lost_volume = area_concave * plate_thickness

            if logger is not None:
                logger.info(f"Lost Volume: {lost_volume * (1000**3)} mm^3")
            hull_cloud_global = self.transform_to_global_coordinates(hull_concave_2d_cloud, mesh1_pca_basis, mesh1_plane_centroid)
            
            
        
        #transform back to global for visualization
        changed_pcl_global = self.transform_to_global_coordinates(changed_pcl_local, mesh1_pca_basis, mesh1_plane_centroid) 

        return lost_volume, changed_pcl_global, hull_cloud_global