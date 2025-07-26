#!/bin/bash

echo "Stopping and disabling service..."
sudo systemctl stop fastapi-server.service
sudo systemctl disable fastapi-server.service

echo "Removing service file..."
sudo rm -f /etc/systemd/system/fastapi-server.service

echo "Reloading systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
