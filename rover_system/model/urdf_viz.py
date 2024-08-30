import subprocess
import sys
from urdfpy import URDF

def convert_and_visualize(xacro_path):
    # Define the output URDF file path
    output_urdf = "output.urdf"
    
    # Convert the Xacro file to a URDF file using the ROS 2 xacro command
    try:
        subprocess.run(["ros2", "run", "xacro", "xacro", xacro_path, "-o", output_urdf], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error converting Xacro to URDF: {e}")
        sys.exit(1)

    # Load and visualize the URDF using urdfpy
    try:
        robot = URDF.load(output_urdf)
        robot.show()
    except Exception as e:
        print(f"Error loading or visualizing URDF: {e}")
        sys.exit(1)

if __name__ == "__main__":
    # Check if the script has been provided with a Xacro file path as an argument
    if len(sys.argv) != 2:
        print("Usage: python visualize_xacro.py <path_to_xacro_file>")
        sys.exit(1)

    # Get the Xacro file path from the command line argument
    xacro_path = sys.argv[1]

    # Call the function to convert and visualize the Xacro file
    convert_and_visualize(xacro_path)
