#!/bin/bash

echo "🛑 Zastavuji a odstraňuji robot-cameras.service..."
sudo systemctl stop robot-cameras.service
sudo systemctl disable robot-cameras.service
sudo rm -f /etc/systemd/system/robot-cameras.service
