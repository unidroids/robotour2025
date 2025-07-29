#!/bin/bash
set -e

SERVICE_FILE="/etc/systemd/system/zeroconf.service"
LOG_DIR="/robot/data/logs/fastapi"
LOG_FILE="$LOG_DIR/zeroconf.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service zeroconf.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Zeroconf (Bonjour) publisher for FastAPI
After=network.target fastapi-server.service
Requires=fastapi-server.service

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/server

Environment=PYTHONUNBUFFERED=1
ExecStart=/opt/projects/robotour/venv-robotour/bin/python bonjour.py

StandardOutput=append:$LOG_FILE
StandardError=append:$LOG_FILE

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling zeroconf.service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now zeroconf.service

echo "✅ Zeroconf service running. View logs with:"
echo "   tail -f $LOG_FILE"
