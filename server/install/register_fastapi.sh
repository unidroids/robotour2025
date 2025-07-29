#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/fastapi-server.service"
LOG_DIR="/robot/data/logs/fastapi"
LOG_FILE="$LOG_DIR/fastapi.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service fastapi-server"

sudo tee "$SERVICE_PATH" > /dev/null <<EOF
[Unit]
Description=FastAPI server for Robotour
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/server

# ▸ Volitelně uvolni port 8080 před startem
#ExecStartPre=-/usr/bin/fuser -k 8080/tcp

Environment=PYTHONUNBUFFERED=1
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/uvicorn main:app --host 0.0.0.0 --port 8080

StandardOutput=append:$LOG_FILE
StandardError=append:$LOG_FILE

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now fastapi-server.service

echo "✅ Service fastapi-server is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
