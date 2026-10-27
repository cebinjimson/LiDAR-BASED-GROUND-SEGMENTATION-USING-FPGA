import numpy as np
import open3d as o3d

# 1. Load Data
file_path = r"C:\Users\cebin\Downloads\2011_09_26_drive_0005_sync\2011_09_26\2011_09_26_drive_0005_sync\velodyne_points\data\0000000011.bin"
raw_points = np.fromfile(file_path, dtype=np.float32).reshape(-1, 4)

# 2. ROI Filter
x_min, x_max = 0.0, 50.0
y_min, y_max = -25.0, 25.0
z_min_bound, z_max_bound = -3.0, 2.0

roi_mask = (
    (raw_points[:, 0] >= x_min) & (raw_points[:, 0] < x_max) &
    (raw_points[:, 1] >= y_min) & (raw_points[:, 1] < y_max) &
    (raw_points[:, 2] >= z_min_bound) & (raw_points[:, 2] < z_max_bound)
)
pts_roi = raw_points[roi_mask]

# Baseline Float Classification
cell_size = 0.5
grid_x_size = 100
grid_y_size = 100

gx_flt = np.clip(np.floor((pts_roi[:, 0] - x_min) / cell_size).astype(np.int32), 0, grid_x_size - 1)
gy_flt = np.clip(np.floor((pts_roi[:, 1] - y_min) / cell_size).astype(np.int32), 0, grid_y_size - 1)

z_grid_flt = np.full((grid_x_size, grid_y_size), np.inf, dtype=np.float32)
for i in range(len(pts_roi)):
    xi, yi, zi = gx_flt[i], gy_flt[i], pts_roi[i, 2]
    if zi < z_grid_flt[xi, yi]:
        z_grid_flt[xi, yi] = zi

float_ground_mask = (pts_roi[:, 2] - z_grid_flt[gx_flt, gy_flt]) <= 0.20

# 3. Fixed-Point Emulation (Q12.4, 16-bit Signed)
SCALE = 16  # 2^4

x_fx = np.int16(np.round(pts_roi[:, 0] * SCALE))
y_fx = np.int16(np.round(pts_roi[:, 1] * SCALE))
z_fx = np.int16(np.round(pts_roi[:, 2] * SCALE))

X_MIN_FX = np.int16(0 * SCALE)
Y_MIN_FX = np.int16(-25 * SCALE)
T_GROUND_FX = np.int16(np.round(0.20 * SCALE))

gx_raw = ((x_fx - X_MIN_FX) >> 3).astype(np.int32)
gy_raw = ((y_fx - Y_MIN_FX) >> 3).astype(np.int32)

gx_fx = np.clip(gx_raw, 0, grid_x_size - 1)
gy_fx = np.clip(gy_raw, 0, grid_y_size - 1)

Z_MAX_INIT = np.int16(32767)
z_grid_fx = np.full((grid_x_size, grid_y_size), Z_MAX_INIT, dtype=np.int16)

for i in range(len(pts_roi)):
    xi, yi, zi = gx_fx[i], gy_fx[i], z_fx[i]
    if zi < z_grid_fx[xi, yi]:
        z_grid_fx[xi, yi] = zi

z_min_val = z_grid_fx[gx_fx, gy_fx]
fixed_ground_mask = (z_fx - z_min_val) <= T_GROUND_FX

# 4. Metrics Output
total_roi_points = len(pts_roi)
num_ground = int(fixed_ground_mask.sum())
num_non_ground = total_roi_points - num_ground
accuracy = ((float_ground_mask == fixed_ground_mask).sum() / total_roi_points) * 100.0

print(f"Total ROI Points Evaluated : {total_roi_points}")
print(f"Ground Points (Fixed-Point): {num_ground} ({num_ground / total_roi_points * 100:.2f}%)")
print(f"Non-Ground Points (FPGA)   : {num_non_ground} ({num_non_ground / total_roi_points * 100:.2f}%)")
print(f"Match with Float Baseline  : {accuracy:.2f}%")

# 5. Open3D Interactive Rendering
pcd = o3d.geometry.PointCloud()
pcd.points = o3d.utility.Vector3dVector(pts_roi[:, :3].astype(np.float64))

# Color setup: Forest Green for Ground, Crimson Red for Obstacles
colors = np.zeros((total_roi_points, 3), dtype=np.float64)
colors[fixed_ground_mask] = [0.13, 0.55, 0.13]  # Green
colors[~fixed_ground_mask] = [0.86, 0.08, 0.24] # Red
pcd.colors = o3d.utility.Vector3dVector(colors)

# Render coordinate frame (Red=X, Green=Y, Blue=Z)
coord_axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=3.0, origin=[0, 0, 0])

vis = o3d.visualization.Visualizer()
vis.create_window(window_name="Open3D - FPGA Fixed-Point LiDAR Segmentation", width=1280, height=720)
vis.add_geometry(pcd)
vis.add_geometry(coord_axes)

# Rendering settings
opt = vis.get_render_option()
opt.background_color = np.asarray([0.05, 0.05, 0.05])  # Dark background
opt.point_size = 2.0

vis.run()
vis.destroy_window()