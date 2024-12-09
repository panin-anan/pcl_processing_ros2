# SAMXL_pointcloud_area_detection

## Overview
Code to perform volume loss calculation between two point clouds: from before grinding and from after grinding, scanned by laser line scanner.
The node works in conjuction with `data_gathering` repositories from `git@github.com:Luka140/data_gathering.git`.

## Installation

#### Dependencies (INCOMPLETE)
- ROS2 Humble
- pyads
- open3d
- sklearn
- scipy
- concave_hull

```bash
pip install pyads
pip install open3d==0.18.0
pip install numpy==1.24.0
pip install scikit-learn==1.5.1
pip install scipy==1.8.0
pip install git+https://github.com/panin-ananwa/concave_hull.git

sudo apt-get update
sudo apt install ros-humble-rosbag2-storage-mcap
```

#### Building
To build from source, clone the latest version from this repository into your workspace along with the following repositories:
- `data_gathering_msgs` : Server for the Service RequestPCLVolumeDiff

```bash
git clone git@github.com:Luka140/data_gathering_msgs.git
git clone git@github.com:panin-ananwa/pcl_processing_ros2.git

```
and compile the packages:
To build, after cloning the package:
```bash
colcon build --symlink-install
install/setup.bash
```


## Nodes
### pcl_processing
A node that automates the volume loss calculation process. The node is a service server which waits for the Service Request, `RequestPCLVolumeDiff`, before 
the node computes the volume loss from the latest two point clouds stored in a service client. The client is created either in `test_coordinator` node from `data_gathering` package 
or in `pcl_publush_manual` node for manual volume loss calculation.

The volume loss is calculated from the point cloud difference between two point clouds, clouds from before and after grinding, using the following process:
Note: For now, the algorithm is only applicable to flat plate

1. Uses RANSAC to segment the two set of point clouds and determine the plane parameters of the plate, along with DBSCAN clustering to filter out points unrelated to the sample plate.
2. Project points onto the same plane and uses KDTree to search for nearest neighbours for difference in point-point distance:
   Any points which are a certain `laserline_threshold` or `feedaxis_threshold` distance away from the post grind points are points that have disappeared from compared to the pre-grind point cloud.
3. DBSCAN clustering again to filter out outliers
4. Calculate volume lost from area * `plate_thickness` in which the area is calculated using concave hull algorithm to find the area from the latest clustered 2D point cloud.

Parameters:
- `dist_threshold`: out of plane distance to include points on the plane of the sample plate from RANSAC
- `cluster_neighbor`: minimum number of points for a cluster in DBSCAN clustering
- `plate_thickness`: sample plate thickness to be used to calculate volume loss
- `plane_error_allowance`: allowable errors in plane angle between two point clouds before throwing error.
- `clusterscan_eps`: minimum distance for grouping cluster in point difference clustering algorithm
- `laserline_threshold`: minimum distance threshold for detecting point cloud difference in the laserline axis (finer resolution axis)
- `feedaxis_threshold`: minimum distance threshold for detecting point cloud difference in the feedaxis axis (coarser resolution axis)
- `concave_resolution`: resolution for concave hull: high = tight-knitted area, low = convex hull
- `filter_down_size`: voxel down size of scanned data for clustering algorithm to speed up clustering process
- `belt_width_threshold`: threshold for checking a valid pointcloud difference detection based on grinding belt standard width.

Topics:
- `/scanner/volume` [Float32Stamped](https://github.com/Luka140/stamped_std_msgs/blob/main/msg/Float32Stamped.msg): Publishes the calculated volume lost during the grinding process.
- `/grinded_cloud` [PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html): Publishes the point cloud of the grinded surface after volume loss calculation.
- `/concave_hull_lines` [Marker](http://docs.ros.org/en/api/visualization_msgs/html/msg/Marker.html): Publishes concave hull lines as markers for visualization of the calculated hull from the point cloud.

Services:
- `calculate_volume_lost` [RequestPCLVolumeDiff](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/RequestPCLVolumeDiff.srv): A service that calculates the volume lost between two point clouds, taking initial and final point clouds as inputs, and returning the volume difference and the processed point cloud.


### pcl_publish_manual
A node that can be used to manually publish saved point cloud to pcl_processing, in order to manually evaluate tested data/point clouds.
use Key P to initiate UI and load desired pairs of point cloud for evaluation. The point clouds are also visualize in rviz.

Parameters:
- `dist_threshold`: out of plane distance to include points on the plane of the sample plate from RANSAC
- `cluster_neighbor`: minimum number of points for a cluster in DBSCAN clustering
- `plate_thickness`: sample plate thickness to be used to calculate volume loss
- `plane_error_allowance`: allowable errors in plane angle between two point clouds before throwing error.
- `clusterscan_eps`: minimum distance for grouping cluster in point difference clustering algorithm
- `laserline_threshold`: minimum distance threshold for detecting point cloud difference in the laserline axis (finer resolution axis)
- `feedaxis_threshold`: minimum distance threshold for detecting point cloud difference in the feedaxis axis (coarser resolution axis)
- `concave_resolution`: resolution for concave hull: high = tight-knitted area, low = convex hull
- `filter_down_size`: voxel down size of scanned data for clustering algorithm to speed up clustering process
- `belt_width_threshold`: threshold for checking a valid pointcloud difference detection based on grinding belt standard width.

Topics:
- `/combined_cloud` [PointCloud2](http://docs.ros.org/en/api/sensor_msgs/html/msg/PointCloud2.html): Publishes the point cloud to rviz when key P is pressed and point clouds loaded

Services:
- `calculate_volume_lost` [RequestPCLVolumeDiff](https://github.com/Luka140/data_gathering_msgs/blob/main/srv/RequestPCLVolumeDiff.srv): A service that calculates the volume lost between two point clouds, taking initial and final point clouds as inputs, and returning the volume difference and the processed point cloud.
