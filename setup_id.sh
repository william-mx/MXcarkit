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

# Ensure serial number is only numbers
if ! [[ "$SERIAL_NUMBER" =~ ^[0-9]+$ ]]; then
    echo "Serial number should only contain digits."
    exit 1
fi

# Format the device ID
DEVICE_ID="mxck$(printf "%04d" "$SERIAL_NUMBER")"
echo "Setting device id to $DEVICE_ID..."

echo "Setting hostname to 'mxck'..."
hostnamectl set-hostname mxck

# Persist hostname across reboots
echo "mxck" > /etc/hostname
sed -i "s/127.0.1.1 .*/127.0.1.1 mxck/" /etc/hosts

echo "Setting autologin for user '$USERNAME'..."
if [ -f /etc/gdm3/custom.conf ]; then
    sed -i '/^\[daemon\]/a AutomaticLoginEnable=true\nAutomaticLogin='"$USERNAME" /etc/gdm3/custom.conf
elif [ -f /etc/lightdm/lightdm.conf ]; then
    sed -i '/^\[Seat:\*\]/a autologin-user='"$USERNAME" /etc/lightdm/lightdm.conf
else
    echo "Could not find display manager configuration. Manual setup required."
    exit 1
fi

# Store DEVICE_ID in system-wide environment
echo "DEVICE_ID=$DEVICE_ID" > /etc/environment
echo "export DEVICE_ID=$DEVICE_ID" >> /home/$USERNAME/.bashrc

