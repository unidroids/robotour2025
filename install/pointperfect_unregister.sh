#!/bin/bash
set -e

echo "Stopping and disabling service..."
sudo systemctl stop robot-pointperfect.service || true
sudo systemctl disable robot-pointperfect.service || true

echo "Removing service file..."
sudo rm -f /etc/systemd/system/robot-pointperfect.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload

echo "✅ robot-pointperfect unregistered."
