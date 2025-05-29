# rover-2025

To build the image and run the container, go into the devops directory. 

run the following:

- sudo docker compose up --build

Then open a new terminal and run:

- sudo docker run -it devops-ros

Then, build the packages by running:

- colcon build
- source install/setup.bash


**NOTE** 

Docker works by reading a Dockerfile and building an image based on that file. This image acts as a blueprint for creating a container. The container is an instance of the image that contains all the dependencies, libraries, and configurations needed to run the application or system. 