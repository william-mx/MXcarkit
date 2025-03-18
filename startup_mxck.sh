#!/bin/bash

# Load DEVICE_ID from system-wide environment
source /etc/environment

# Default to mxck0000 if DEVICE_ID is not set
if [[ -z "$DEVICE_ID" ]]; then
    DEVICE_ID="mxck0000"
fi

echo "Starting WiFi hotspot with SSID and password: $DEVICE_ID"

# Set up a hotspot with DEVICE_ID as SSID and password
nmcli dev wifi hotspot ifname wlan0 ssid "$DEVICE_ID" password "$DEVICE_ID"








