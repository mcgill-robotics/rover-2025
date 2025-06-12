# Docker

## What is Docker

Docker is used to run software applications which require specific software packages/libraries/operating systems inside of **containers** which are instances of **Docker Images**. 

**Docker Images** are snapshots of a software environment that include both the software dependencies and the operating system itself. It is a streamlined approach to sharing the development environment between other users so that not everyone needs to manually ensure they have the same exact setup to run someone else's code. This is important as software packages often deceprate over time which can lead to a lot of dependency problems, but Docker allows us to select the specific packages, and their versions, we need to run our applications.

Building the Docker Image requires creating and reading from a **Dockerfile** that acts as a blueprint for creating your desired environment via a series of specific instructions. 

To actually use the environment in the Docker Image and run some code, we need to create a **container**, which can be viewed as an instance of the image we already built. Inside the container, we can effectively do everything we would want as if we were running the environment natively. However, since each container we spawn is an instance of its Docker Image, any changes made inside the container will not persist if we close and open a container. For our purposes, to use our local repo inside the container, we mount it onto the container when we create it so that we can make changes to our code inside the container.

## Setup Docker

### Linux
If you are using a Linux OS like Ubuntu, you can use our `linux_docker_setup.sh` shell script to automatically download and setup docker for you!

### Windows/macOS

You can download docker-desktop via the following [link](https://www.docker.com/products/docker-desktop/)



## Building the Docker Image
To build the docker image, run the following command in the **docker**  directory. This will take a while to build, but you only need to do it once unless you change the dockerfile.

```
sudo docker compose --build
```

## Running the Docker Container

To access the container based on the image we made, open a terminal in the **docker** directory and run :

```
sudo docker compose up
```

This will create a container using the setup flags specified in `docker-compose.yml`. While this container is running, you can access it by:
- Opening a new terminal
- Run the following command
```
sudo docker compose exec halley bash
```

This should place you in the rover25_ws directory. Afterwards you should run the following two commands:

```
colcon build
source install/setup.bash
```

Now you should able to work and access our repo as if you have your own linux setup:)

