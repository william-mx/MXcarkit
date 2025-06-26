#!/bin/bash

# -----------------------------------------------
# Start Portainer container (if not already running)
# -----------------------------------------------

# Check if a container named "portainer" already exists
if docker ps -a --format '{{.Names}}' | grep -q "^portainer$"; then
    echo "Portainer container already exists. Attempting to start it..."
    docker start portainer
else
    echo "Portainer container not found. Creating and starting a new instance..."
    
    # Run a new Portainer container with:
    # - persistent volume for configuration
    # - access to the local Docker socket
    # - web UI exposed on port 9000
    # - automatic restart enabled
    docker run -d \
       --name portainer \
       --restart=always \
       -p 9000:9000 \
       -v /var/run/docker.sock:/var/run/docker.sock \
       -v portainer_data:/data \
       portainer/portainer-ce
fi

# Get the IP address of wlan0
JETSON_IP=$(ip -4 addr show wlan0 | grep -oP '(?<=inet\s)\d+(\.\d+){3}')

# Print access information
if [[ -n "$JETSON_IP" ]]; then
    echo "Portainer is now accessible at: http://$JETSON_IP:9000"
else
    echo "Portainer is running, but no IP address was found on wlan0. Is Wi-Fi connected?"
fi
