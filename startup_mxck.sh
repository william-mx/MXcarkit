#!/bin/bash

# Load DEVICE_ID from system-wide environment
source /etc/environment

# Default to mxck0000 if DEVICE_ID is not set
if [[ -z "$DEVICE_ID" ]]; then
    DEVICE_ID="mxck0000"
fi

# Set target Wi-Fi network
TARGET_SSID="MXcarkit_Network_5G"
TARGET_PASSWORD="39532305"

echo "Looking for network: $TARGET_SSID"

# Scan for available Wi-Fi networks
AVAILABLE=$(nmcli -t -f SSID dev wifi list | grep "^${TARGET_SSID}$")

if [[ -n "$AVAILABLE" ]]; then
    echo "Connecting to $TARGET_SSID..."
    nmcli dev wifi connect "$TARGET_SSID" password "$TARGET_PASSWORD"
else
    echo "Network $TARGET_SSID not found. Starting hotspot with SSID: $DEVICE_ID"
    nmcli dev wifi hotspot ifname wlan0 ssid "$DEVICE_ID" password "$DEVICE_ID"
fi

