import os
import glob

folder = "pointclouds"

files = glob.glob(os.path.join(folder, "*.pcd")) + glob.glob(os.path.join(folder, "*.ply"))

if not files:
    print(f"No point cloud files found in '{folder}/'")
else:
    for f in files:
        os.remove(f)
    print(f"Deleted {len(files)} file(s) from '{folder}/'")