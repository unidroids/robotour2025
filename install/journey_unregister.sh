#!/bin/bash

echo "🛑 Zastavuji a odstraňuji robot-journey.service..."
sudo systemctl stop robot-journey.service
sudo systemctl disable robot-journey.service
sudo rm -f /etc/systemd/system/robot-journey.service
