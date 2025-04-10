import open3d as o3d
import numpy as np
import copy

# Generate a synthetic point cloud (a sphere)
mesh = o3d.geometry.TriangleMesh.create_sphere(radius=1.0)
mesh.compute_vertex_normals()
pcd1 = mesh.sample_points_poisson_disk(500)

# Create a transformed copy of the original point cloud
pcd2 = copy.deepcopy(pcd1)
angle = np.pi / 4  # 45 degree rotation
R = pcd2.get_rotation_matrix_from_xyz((angle, 0, 0))  # rotate around x-axis
pcd2.rotate(R, center=(0, 0, 0))
pcd2.translate((0.5, 0.5, 0))  # translate it for added challenge

# Visualize before ICP
o3d.visualization.draw_geometries([
    pcd1.paint_uniform_color([1, 0, 0]),
    pcd2.paint_uniform_color([0, 1, 0])
])

# Apply ICP
threshold = 1.0
reg = o3d.pipelines.registration.registration_icp(
    pcd2, pcd1, threshold, np.eye(4),
    o3d.pipelines.registration.TransformationEstimationPointToPoint()
)

# Show ICP metrics
print("ICP Fitness:", reg.fitness)
print("ICP Inlier RMSE:", reg.inlier_rmse)
print("ICP Transformation:")
print(reg.transformation)

# Apply transformation
pcd2_aligned = copy.deepcopy(pcd2)
pcd2_aligned.transform(reg.transformation)

# Visualize after ICP
o3d.visualization.draw_geometries([
    pcd1.paint_uniform_color([1, 0, 0]),
    pcd2_aligned.paint_uniform_color([0, 1, 0])
])
