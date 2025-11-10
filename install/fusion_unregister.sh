#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-fusion.service || true
sudo systemctl disable robot-fusion.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-fusion.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-fusion unregistered."
