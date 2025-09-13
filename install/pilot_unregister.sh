#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-pilot.service || true
sudo systemctl disable robot-pilot.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-pilot.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-pilot unregistered."
