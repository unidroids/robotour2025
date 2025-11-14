#!/bin/bash
set -e

SERVICE_NAME="bluetooth-gamepad-autoconnect"
SERVICE_PATH="/etc/systemd/system/$SERVICE_NAME.service"

LOG_DIR="/data/logs/gamepad"
LOG_FILE="$LOG_DIR/autoconnect.log"

echo "📁 Vytvářím logovací adresář..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Vytvářím systemd službu $SERVICE_NAME"

sudo tee "$SERVICE_PATH" > /dev/null <<EOF
[Unit]
Description=Automatic Bluetooth gamepad autoconnect
After=bluetooth.service
Requires=bluetooth.service

[Service]
Type=simple
User=user
WorkingDirectory=/opt/projects/robotour/gamepad/bluetooth

Environment=PYTHONUNBUFFERED=1
ExecStart=/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/gamepad/bluetooth/gamepad_autoconnect.py

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
sudo systemctl enable --now "$SERVICE_NAME.service"

echo "✅ Service $SERVICE_NAME je aktivní. Logy:"
echo "   tail -f $LOG_FILE"
echo "   # nebo:"
echo "   journalctl -u $SERVICE_NAME -f"
