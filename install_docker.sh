#!/bin/bash

# Ensure the script is run as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

echo "Updating package lists..."
sudo apt update

echo "Installing Docker..."
sudo apt install -y docker.io

echo "Adding current user to the Docker group..."
sudo usermod -aG docker $USER

echo "Installing NVIDIA Container Runtime..."
sudo apt install -y nvidia-container-runtime

echo "Configuring Docker to use NVIDIA runtime..."
DOCKER_CONFIG='/etc/docker/daemon.json'

# Create or update the Docker daemon config
if [ -f "$DOCKER_CONFIG" ]; then
    sudo jq '.runtimes.nvidia = { "path": "nvidia-container-runtime", "runtimeArgs": [] } | .default-runtime = "nvidia"' "$DOCKER_CONFIG" > tmpfile && sudo mv tmpfile "$DOCKER_CONFIG"
else
    echo '{
        "runtimes": {
            "nvidia": {
                "path": "nvidia-container-runtime",
                "runtimeArgs": []
            }
        },
        "default-runtime": "nvidia"
    }' | sudo tee "$DOCKER_CONFIG" > /dev/null
fi

echo "Restarting Docker..."
sudo systemctl restart docker
sudo systemctl enable docker

echo "Installation complete. Please reboot for changes to take full effect."
