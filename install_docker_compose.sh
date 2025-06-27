#!/bin/bash

set -e

# Variables
COMPOSE_VERSION="v2.37.3"
ARCH="aarch64"
INSTALL_DIR="$HOME/.docker/cli-plugins"

echo "[INFO] Installing Docker Compose $COMPOSE_VERSION for $ARCH..."

# Step 1: Create plugin directory
mkdir -p "$INSTALL_DIR"

# Step 2: Download binary
curl -SL "https://github.com/docker/compose/releases/download/${COMPOSE_VERSION}/docker-compose-linux-${ARCH}" \
  -o "$INSTALL_DIR/docker-compose"

# Step 3: Make executable
chmod +x "$INSTALL_DIR/docker-compose"

# Step 4: Verify
echo "[INFO] Docker Compose version:"
docker compose version || echo "Make sure Docker is installed and you restart your terminal session."

echo "[DONE] Docker Compose installed in: $INSTALL_DIR/docker-compose"
