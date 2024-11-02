import os
import subprocess


def check_build():
    """
    """
    #os.chdir("..") #change to workspace directory
    subprocess.run("pwd") #check which current location
    package = input("Package name: ")
    command = ["colcon", "build",  "--packages-select", package]

    #Try building any package
    try:
        subprocess.run(command, check=True)
        print("Package built successfully")
    except subprocess.CalledProcessError as e:
        print("Package build failed")
        print("Error:", e)



if __name__ == "__main__":
    check_build()
