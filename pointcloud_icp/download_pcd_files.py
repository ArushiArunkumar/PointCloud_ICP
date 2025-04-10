import open3d as o3d
import numpy as np

# Generate random source point cloud
np.random.seed(42)
source_points = np.random.rand(100, 3)
source_pcd = o3d.geometry.PointCloud()
source_pcd.points = o3d.utility.Vector3dVector(source_points)

# Apply a transformation to create the target point cloud
R = source_pcd.get_rotation_matrix_from_xyz((0.5, 0.0, 0.0))  # rotate around x-axis
T = np.array([0.2, 0.3, 0.1])
target_pcd = source_pcd.rotate(R, center=(0, 0, 0)).translate(T)

# Save to disk
o3d.io.write_point_cloud("cloud1.pcd", source_pcd)
o3d.io.write_point_cloud("cloud2.pcd", target_pcd)

print("PCD files saved as cloud1.pcd and cloud2.pcd")
