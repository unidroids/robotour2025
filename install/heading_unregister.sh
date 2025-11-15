#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-heading.service || true
sudo systemctl disable robot-heading.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-heading.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-heading unregistered."
