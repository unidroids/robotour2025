#!/bin/bash

echo "Stopping and disabling service..."
sudo systemctl stop bluetooth-gamepad-autoconnect.service
sudo systemctl disable bluetooth-gamepad-autoconnect.service

echo "Removing service file..."
sudo rm -f /etc/systemd/system/bluetooth-gamepad-autoconnect.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
