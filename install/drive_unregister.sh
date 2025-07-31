#!/bin/bash

echo "Stopping and disabling service..."
sudo systemctl stop robot-drive.service
sudo systemctl disable robot-drive.service

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-drive.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
