#!/bin/bash

# Ensure script is run as root
if [[ $EUID -ne 0 ]]; then
   echo "This script must be run as root"
   exit 1
fi

# Define fixed username
USERNAME="mxck"

# Get serial number input
read -p "Enter the serial number: " SERIAL_NUMBER

# Validate input
if [[ -z "$SERIAL_NUMBER" ]]; then
    echo "Serial number cannot be empty."
    exit 1
fi

# Ensure serial number is only digits
if ! [[ "$SERIAL_NUMBER" =~ ^[0-9]+$ ]]; then
    echo "Serial number should only contain digits."
    exit 1
fi

# Format the device ID and hostname
DEVICE_ID="mxck$(printf "%04d" "$SERIAL_NUMBER")"
NETWORK_NAME="${DEVICE_ID}-net"

echo "Setting device ID and hostname to $DEVICE_ID..."

# Set and persist hostname
hostnamectl set-hostname "$DEVICE_ID"
echo "$DEVICE_ID" > /etc/hostname
sed -i "s/127.0.1.1 .*/127.0.1.1 $DEVICE_ID/" /etc/hosts

# Configure autologin for the user
echo "Setting autologin for user '$USERNAME'..."
if [ -f /etc/gdm3/custom.conf ]; then
    sed -i '/^\[daemon\]/a AutomaticLoginEnable=true\nAutomaticLogin='"$USERNAME" /etc/gdm3/custom.conf
elif [ -f /etc/lightdm/lightdm.conf ]; then
    sed -i '/^\[Seat:\*\]/a autologin-user='"$USERNAME" /etc/lightdm/lightdm.conf
else
    echo "Could not find display manager configuration. Manual setup required."
    exit 1
fi

# Store DEVICE_ID in environment
echo "DEVICE_ID=$DEVICE_ID" > /etc/environment
echo "export DEVICE_ID=$DEVICE_ID" >> /home/$USERNAME/.bashrc

# ---- Create Docker network without fixed subnet/gateway ----
echo "Checking Docker network '$NETWORK_NAME'..."

if docker network ls --format '{{.Name}}' | grep -q "^${NETWORK_NAME}$"; then
    echo "Docker network '$NETWORK_NAME' already exists."
else
    echo "Creating Docker network '$NETWORK_NAME' with auto-assigned subnet/gateway..."
    docker network create \
      --driver=bridge \
      "$NETWORK_NAME"
    echo "Docker network '$NETWORK_NAME' created successfully."
fi
