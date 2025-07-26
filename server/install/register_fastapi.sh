#!/bin/bash

SERVICE_PATH="/etc/systemd/system/fastapi-server.service"

echo "Creating systemd service..."

sudo tee $SERVICE_PATH > /dev/null <<EOF
[Unit]
Description=FastAPI server for Robotour
After=network.target

[Service]
User=user
WorkingDirectory=/robot/opt/projects/robotour/server
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/uvicorn main:app --host 0.0.0.0 --port 8080
Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "Enabling and starting service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now fastapi-server.service
