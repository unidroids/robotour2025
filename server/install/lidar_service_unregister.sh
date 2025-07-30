#!/bin/bash

echo "🛑 Zastavuji a odstraňuji robot-lidar.service..."
sudo systemctl stop robot-lidar.service
sudo systemctl disable robot-lidar.service
sudo rm -f /etc/systemd/system/robot-lidar.service
