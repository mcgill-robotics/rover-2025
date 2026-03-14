import pyrealsense2 as rs
import numpy as np
import os
import time

def save_ply(points_3d, colors, filename):
    valid = points_3d[:, 2] > 0
    points_3d = points_3d[valid]
    colors = colors[valid]

    with open(filename, 'w') as f:
        f.write("ply\n")
        f.write("format ascii 1.0\n")
        f.write(f"element vertex {len(points_3d)}\n")
        f.write("property float x\n")
        f.write("property float y\n")
        f.write("property float z\n")
        f.write("property uchar red\n")
        f.write("property uchar green\n")
        f.write("property uchar blue\n")
        f.write("end_header\n")

        for point, color in zip(points_3d, colors):
            r, g, b = int(color[0]), int(color[1]), int(color[2])
            f.write(f"{point[0]:.4f} {point[1]:.4f} {point[2]:.4f} {r} {g} {b}\n")

# Configure streams
pipeline = rs.pipeline()
config = rs.config()
align = rs.align(rs.stream.color)
pc = rs.pointcloud()

pipeline.start(config)
os.makedirs("pointclouds_ply", exist_ok=True)

print("Warming up camera...")
for i in range(30):
    pipeline.wait_for_frames()

try:
    frame_count = 0
    last_save = 0
    print("Recording... press Ctrl+C to stop")

    while True:
        frames = pipeline.wait_for_frames()
        aligned_frames = align.process(frames)
        depth_frame = aligned_frames.get_depth_frame()
        color_frame = aligned_frames.get_color_frame()

        if not depth_frame or not color_frame:
            continue

        now = time.time()
        if now - last_save >= 0.1:
            pc.map_to(color_frame)
            points = pc.calculate(depth_frame)

            vertices = np.asanyarray(points.get_vertices()).view(np.float32).reshape(-1, 3)
            tex_coords = np.asanyarray(points.get_texture_coordinates()).view(np.float32).reshape(-1, 2)
            color_image = np.asanyarray(color_frame.get_data())
            h, w = color_image.shape[:2]
            u = np.clip((tex_coords[:, 0] * w).astype(int), 0, w - 1)
            v = np.clip((tex_coords[:, 1] * h).astype(int), 0, h - 1)
            colors = color_image[v, u]

            filename = f"pointclouds_ply/frame_{frame_count:04d}.ply"
            save_ply(vertices, colors, filename)
            print(f"Saved {filename} — {len(vertices)} points")

            frame_count += 1
            last_save = now

finally:
    pipeline.stop()
    print("Stopped.")