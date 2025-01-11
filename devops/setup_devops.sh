#!/usr/bin/env bash
# setup.sh
#
# This script:
#   1) Ensures you have a ROS2 workspace folder at ~/rover25_ws/src
#   2) Clones the rover-2025 repo into that workspace
#   3) Enters the devops/ folder
#   4) Optionally builds the Docker images and runs docker compose up
#
# Feel free to adjust any paths or commands per your environment.

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/debian/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
    "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/debian \
  $(. /etc/os-release && echo "$VERSION_CODENAME") stable" |
    sudo tee /etc/apt/sources.list.d/docker.list >/dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

# User add
sudo groupadd docker
sudo usermod -aG docker $USER
# newgrp docker

# set -e # Exit immediately on any command failure

########################################
# STEP 1: Create the ROS2 Workspace
########################################
echo "[Setup] Creating ~/rover25_ws/src if it doesn't exist..."
mkdir -p ~/rover25_ws/src

########################################
# STEP 2: Clone the rover-2025 Repo
########################################
# If you already have it, consider removing or skipping.
# By default, this will fail if 'rover-2025' folder already exists
# (unless you want to use 'git pull' or handle it differently).
echo "[Setup] Cloning rover-2025 repository into ~/rover25_ws/src..."
git clone --branch devops https://github.com/mcgill-robotics/rover-2025.git ~/rover25_ws/src

########################################
# STEP 3: Enter the devops/ folder
########################################
cd ~/rover25_ws/src/devops

########################################
# STEP 4: Build & Run Docker Compose
########################################
# Adjust the Compose commands depending on if you need to build first or just run.
echo "[Setup] Running docker compose up -d..."
docker compose build # optional step if you want to force rebuild
docker compose up -d # or 'docker-compose up -d' depending on your system

echo "[Setup] Done! The Docker containers should now be running."
