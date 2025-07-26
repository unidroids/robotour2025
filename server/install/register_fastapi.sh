#!/bin/bash

SERVICE_PATH="/etc/systemd/system/fastapi-server.service"
LOG_DIR="/robot/data/logs/fastapi"
LOG_FILE="$LOG_DIR/fastapi.log"

echo "Creating log directory..."
mkdir -p $LOG_DIR
touch $LOG_FILE
chmod 664 $LOG_FILE

echo "Creating systemd service..."

sudo tee $SERVICE_PATH > /dev/null <<EOF
[Unit]
Description=FastAPI server for Robotour
After=network.target

[Service]
User=user
WorkingDirectory=/robot/opt/projects/robotour/server
ExecStart=/bin/bash -c '/robot/opt/projects/robotour/venv-robotour/bin/uvicorn main:app --host 0.0.0.0 --port 8080 >> $LOG_FILE 2>&1'
Restart=always
RestartSec=3
StandardOutput=append:$LOG_FILE
StandardError=append:$LOG_FILE

[Install]
WantedBy=multi-user.target
EOF

echo "Enabling and starting service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now fastapi-server.service
