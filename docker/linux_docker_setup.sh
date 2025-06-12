#!/usr/bin/env bash
# setup.sh
#
# This script:
#   1) Install docker onto your device
#   2) Builds the Docker images and runs docker compose up
#
# Feel free to adjust any paths or commands per your environment.

# Add Docker's official GPG key:
sudo apt-get update
sudo apt-get install ca-certificates curl
sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

# Add the repository to Apt sources:
echo \
  "deb [arch=$(dpkg --print-architecture) signed-by=/etc/apt/keyrings/docker.asc] https://download.docker.com/linux/ubuntu \
  $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}") stable" | \
  sudo tee /etc/apt/sources.list.d/docker.list > /dev/null
sudo apt-get update

sudo apt-get install docker-ce docker-ce-cli containerd.io docker-buildx-plugin docker-compose-plugin

sudo systemctl enable docker
sudo systemctl start docker

# User add
sudo groupadd docker
sudo usermod -aG docker $USER
newgrp docker

########################################
# Build & Run Docker Compose
########################################
# Adjust the Compose commands depending on if you need to build first or just run.
echo "[Setup] Running docker compose up -d..."
docker compose build # optional step if you want to force rebuild
docker compose up -d # or 'docker-compose up -d' depending on your system

echo "[Setup] Done! The Docker containers should now be running."
