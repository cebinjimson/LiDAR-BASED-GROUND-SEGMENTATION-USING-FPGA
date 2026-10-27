from pathlib import Path
import cv2
import numpy as np

# -------------------------------------------------------------
# 1. FILE PATHS SETUP
# -------------------------------------------------------------
calib_dir = Path(r"C:\Users\cebin\Downloads\2011_09_26_calib\2011_09_26")
calib_cam_to_cam_file = calib_dir / "calib_cam_to_cam.txt"
calib_velo_to_cam_file = calib_dir / "calib_velo_to_cam.txt"

data_base_dir = Path(r"C:\Users\cebin\Downloads\2011_09_26_drive_0005_sync\2011_09_26\2011_09_26_drive_0005_sync")
velo_file = data_base_dir / "velodyne_points" / "data" / "0000000011.bin"
image_file = data_base_dir / "image_02" / "data" / "0000000011.png"

# -------------------------------------------------------------
# 2. ROBUST CALIBRATION PARSER
# -------------------------------------------------------------
def read_calib_file(filepath):
    data = {}
    with open(filepath, "r") as f:
        for line in f:
            line = line.strip()
            if not line or ":" not in line:
                continue
            key, val = line.split(":", 1)
            try:
                numeric_vals = np.array([float(x) for x in val.strip().split()])
                if len(numeric_vals) > 0:
                    data[key.strip()] = numeric_vals
            except ValueError:
                continue
    return data

def get_projection_matrices(calib_cam_path, calib_velo_path):
    cam_calib = read_calib_file(calib_cam_path)
    velo_calib = read_calib_file(calib_velo_path)

    R_velo = velo_calib["R"].reshape(3, 3)
    T_velo = velo_calib["T"].reshape(3, 1)
    Tr_velo_to_cam = np.vstack([np.hstack([R_velo, T_velo]), [0, 0, 0, 1]])

    R_rect_00 = np.eye(4)
    R_rect_00[:3, :3] = cam_calib["R_rect_00"].reshape(3, 3)

    P_rect_02 = cam_calib["P_rect_02"].reshape(3, 4)

    return P_rect_02 @ R_rect_00 @ Tr_velo_to_cam

# -------------------------------------------------------------
# 3. LOAD DATA & FIXED-POINT SEGMENTATION (Q12.4)
# -------------------------------------------------------------
print("[1/5] Loading LiDAR binary data...")
raw_points = np.fromfile(str(velo_file), dtype=np.float32).reshape(-1, 4)

x_min, x_max = 0.0, 50.0
y_min, y_max = -25.0, 25.0
z_min_bound, z_max_bound = -3.0, 2.0

roi_mask = (
    (raw_points[:, 0] >= x_min) & (raw_points[:, 0] < x_max) &
    (raw_points[:, 1] >= y_min) & (raw_points[:, 1] < y_max) &
    (raw_points[:, 2] >= z_min_bound) & (raw_points[:, 2] < z_max_bound)
)
pts_roi = raw_points[roi_mask]

SCALE = 16
x_fx = np.int16(np.round(pts_roi[:, 0] * SCALE))
y_fx = np.int16(np.round(pts_roi[:, 1] * SCALE))
z_fx = np.int16(np.round(pts_roi[:, 2] * SCALE))

gx = np.clip(((x_fx - 0) >> 3).astype(np.int32), 0, 99)
gy = np.clip(((y_fx - (-25 * SCALE)) >> 3).astype(np.int32), 0, 99)

z_grid_fx = np.full((100, 100), 32767, dtype=np.int16)
for i in range(len(pts_roi)):
    if z_fx[i] < z_grid_fx[gx[i], gy[i]]:
        z_grid_fx[gx[i], gy[i]] = z_fx[i]

T_GROUND_FX = np.int16(np.round(0.20 * SCALE))
fixed_ground_mask = (z_fx - z_grid_fx[gx, gy]) <= T_GROUND_FX

# -------------------------------------------------------------
# 4. 3D LIDAR TO 2D CAMERA PROJECTION
# -------------------------------------------------------------
print("[2/5] Loading camera frame...")
img = cv2.imread(str(image_file))
if img is None:
    raise FileNotFoundError(f"Image not found at {image_file}.")

img_h, img_w, _ = img.shape
pts_homo = np.hstack([pts_roi[:, :3], np.ones((len(pts_roi), 1))])

print("[3/5] Projecting 3D points to 2D image coordinates...")
P_velo_to_img = get_projection_matrices(calib_cam_to_cam_file, calib_velo_to_cam_file)
pts_2d = (P_velo_to_img @ pts_homo.T).T

depth = pts_2d[:, 2]
u = pts_2d[:, 0] / depth
v = pts_2d[:, 1] / depth

valid_mask = (depth > 0) & (u >= 0) & (u < img_w) & (v >= 0) & (v < img_h)

valid_u = u[valid_mask].astype(np.int32)
valid_v = v[valid_mask].astype(np.int32)
valid_ground = fixed_ground_mask[valid_mask]

print(f"Total points projected onto camera: {len(valid_u)}")
print(f"  - Road points (Green)     : {np.sum(valid_ground)}")
print(f"  - Obstacle points (Red)   : {np.sum(~valid_ground)}")

# -------------------------------------------------------------
# 5. OVERLAY POINTS & EXPORT
# -------------------------------------------------------------
print("[4/5] Rendering projection overlay...")
overlay = img.copy()

for ui, vi in zip(valid_u[valid_ground], valid_v[valid_ground]):
    cv2.circle(overlay, (ui, vi), radius=2, color=(0, 230, 0), thickness=-1)

for ui, vi in zip(valid_u[~valid_ground], valid_v[~valid_ground]):
    cv2.circle(overlay, (ui, vi), radius=2, color=(0, 0, 240), thickness=-1)

blended = cv2.addWeighted(overlay, 0.8, img, 0.2, 0)

# Save result image to current folder so it can be opened directly
output_path = Path("segmented_projection.png").resolve()
cv2.imwrite(str(output_path), blended)
print(f"[5/5] Success! Saved output image to:\n      --> {output_path}")

# Display GUI Window
window_title = "LiDAR Segmentation Overlay"
cv2.namedWindow(window_title, cv2.WINDOW_NORMAL)
cv2.resizeWindow(window_title, 1280, 720)
cv2.imshow(window_title, blended)
cv2.waitKey(1)  # Flushes highgui event queue on Windows

print("\nPress any key in the GUI window to exit...")
cv2.waitKey(0)
cv2.destroyAllWindows()