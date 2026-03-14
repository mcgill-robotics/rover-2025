import pyrealsense2 as rs
import numpy as np
import os
import time

def save_pcd(points_3d, colors, filename):
    valid = points_3d[:, 2] > 0
    points_3d = points_3d[valid]
    colors = colors[valid]

    num_points = len(points_3d)

    with open(filename, 'w') as f:
        f.write("# .PCD v0.7 - Point Cloud Data file format\n")
        f.write("VERSION 0.7\n")
        f.write("FIELDS x y z rgb\n")
        f.write("SIZE 4 4 4 4\n")
        f.write("TYPE F F F F\n")
        f.write("COUNT 1 1 1 1\n")
        f.write(f"WIDTH {num_points}\n")
        f.write("HEIGHT 1\n")
        f.write("VIEWPOINT 0 0 0 1 0 0 0\n")
        f.write(f"POINTS {num_points}\n")
        f.write("DATA ascii\n")

        for point, color in zip(points_3d, colors):
            r, g, b = int(color[0]), int(color[1]), int(color[2])
            # Pack RGB into a single float (PCL convention)
            rgb_int = (r << 16) | (g << 8) | b
            # Reinterpret as float
            rgb_float = np.frombuffer(np.uint32(rgb_int).tobytes(), dtype=np.float32)[0]
            f.write(f"{point[0]:.4f} {point[1]:.4f} {point[2]:.4f} {rgb_float}\n")

# Configure streams
pipeline = rs.pipeline()
config = rs.config()
align = rs.align(rs.stream.color)
pc = rs.pointcloud()

pipeline.start(config)
os.makedirs("pointclouds", exist_ok=True)

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

            filename = f"pointclouds_pcd/frame_{frame_count:04d}.pcd"
            save_pcd(vertices, colors, filename)
            print(f"Saved {filename} — {len(vertices)} points")

            frame_count += 1
            last_save = now

finally:
    pipeline.stop()
    print("Stopped.")