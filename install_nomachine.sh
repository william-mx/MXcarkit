#!/bin/bash

# Ensure script runs as root
if [[ $EUID -ne 0 ]]; then
    echo "This script must be run as root"
    exit 1
fi

# Define download URL and file name
URL="https://download.nomachine.com/download/8.16/Arm/nomachine_8.16.1_1_arm64.deb"
FILENAME="nomachine_8.16.1_1_arm64.deb"
DOWNLOAD_DIR="$HOME/Downloads"

# Create Downloads directory if it doesn't exist
mkdir -p "$DOWNLOAD_DIR"

# Change to Downloads directory
cd "$DOWNLOAD_DIR" || exit

# Download NoMachine
echo "Downloading NoMachine..."
wget -O "$FILENAME" "$URL"

# Check if download was successful
if [[ ! -f "$FILENAME" ]]; then
    echo "Download failed! Exiting..."
    exit 1
fi

# Install NoMachine
echo "Installing NoMachine..."
sudo dpkg -i "$FILENAME"

# Fix missing dependencies if needed
echo "Fixing dependencies..."
sudo apt-get install -f -y

# Cleanup (Optional: Remove the .deb file after installation)
echo "Cleaning up..."
rm -f "$FILENAME"

echo "NoMachine installation complete!"
