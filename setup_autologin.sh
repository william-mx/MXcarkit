echo "Setting up mxck_startup.sh to run at boot..."

# Create a systemd service file for mxck_startup.sh
SERVICE_FILE="/etc/systemd/system/mxck_startup.service"
cat <<EOL > $SERVICE_FILE
[Unit]
Description=Start Jetson Hotspot on Boot
After=network.target

[Service]
ExecStart=/bin/bash /home/$USERNAME/MXcarkit/mxck_startup.sh
Restart=always
User=$USERNAME

[Install]
WantedBy=multi-user.target
EOL

# Reload systemd, enable and start the service
systemctl daemon-reload
systemctl enable mxck_startup.service
systemctl start mxck_startup.service

echo "Setup complete. Please reboot the system for all changes to take effect."