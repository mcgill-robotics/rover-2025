# Cameras Package (ROS 2)

This package provides:
- `usbcam_node`: A ROS2 node to publish frames from a USB camera
- `image_subscriber`: A basic node to subscribe and process image topics

---

## 📁 Package Structure

```
cameras/
├── cameras/
│   ├── __init__.py
│   ├── usbcam_node.py
│   └── image_subscriber.py
├── setup.py
├── package.xml
├── resource/
│   └── cameras
├── launch/
│   └── camera_launch.py (optional)
└── requirements.txt (optional)
```

---

## 🧱 Setup Instructions

### 1. Clone the Repo

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
git clone https://github.com/mcgill-robotics/rover-2025.git
cd ~/ros2_ws
```

### 2. Create & Activate Python Virtual Environment (outside `ros2_ws`)

```bash
cd ~
python3 -m venv ros2_venv
source ~/ros2_venv/bin/activate
```

(Optional) Add this to your `~/.bashrc`:

```bash
source ~/ros2_venv/bin/activate
source ~/ros2_ws/install/setup.bash
```

### 3. Install Dependencies

```bash
pip install -r ~/ros2_ws/src/rover-2025/requirements.txt
pip install catkin_pkg colcon-common-extensions empy numpy<2.0
```

> ⚠️ Use `numpy<2.0` to avoid compatibility issues with `cv_bridge` and OpenCV.

### 4. Build the Workspace

```bash
cd ~/ros2_ws
colcon build
source install/setup.bash
```

---

## 📷 Check Your Camera Info

Before launching the camera node, find your USB camera name or device path:

```bash
v4l2-ctl --list-devices
```

Example output:

```
USB 2.0 Camera: USB Camera (usb-0000:00:05.0-3):
    /dev/video0
```

---

## 🚀 Running the Nodes

### A. Run `usbcam_node`

```bash
ros2 run cameras usbcam_node --cam-name /dev/video0
or
ros2 run cameras usbcam_node --cam-name "USB 2.0 Camera"
```

> Use your actual device path from the `v4l2-ctl` output.

#### View Published Topics:

```bash
ros2 topic list
ros2 topic echo /image_raw/compressed
```

### B. Run `image_subscriber`

```bash
ros2 run cameras image_subscriber
```

> This will subscribe to the camera topic and process the images.

---

## 🧪 Optional: Launch File

Create a launch file in `launch/camera_launch.py`:

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cameras',
            executable='usbcam_node',
            name='usbcam_node',
            output='screen',
            parameters=[{'cam_name': '/dev/video0'}]
        )
    ])
```

Then run it with:

```bash
ros2 launch cameras camera_launch.py
```

---

## 🧠 Notes

- `usbcam_node` should define a `main()` method in `cameras/usbcam_node.py`
- The `setup.py` must include `entry_points` like:

```python
entry_points={
    'console_scripts': [
        'usbcam_node = cameras.usbcam_node:main',
        'image_subscriber = cameras.image_subscriber:main',
    ],
}
```

---

## 🛠 Troubleshooting

### Error: `ModuleNotFoundError: No module named 'catkin_pkg'`

→ Run: `pip install catkin_pkg`

### Error: `cv_bridge` NumPy crash

→ Run: `pip install "numpy<2.0"`

### Error: `No executable found`

→ Make sure you rebuilt with `colcon build` and `setup.py` has correct entry point

---

## 👤 Maintainers

- Kevin (kevin@todo.todo)
- McGill Robotics Team

---

## 📜 License

TODO: License declaration