#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-pilot.service"
LOG_DIR="/robot/data/logs/pilot"
LOG_FILE="$LOG_DIR/pilot.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service robot-pilot"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot Pilot (autopilot) for Robotour
Wants=network-online.target
After=network-online.target
# Volitelně lze zapnout závislosti na jiných službách:
# Wants=robot-gnss.service robot-pointperfect.service
# After=robot-gnss.service robot-pointperfect.service

# Pomůže zachytit chybějící soubory srozumitelněji než CHDIR fail
ConditionPathExists=/opt/projects/robotour/pilot/main.py

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/pilot

# před spuštěním ukonči libovolný proces, který drží port 9008
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9008/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
# Volitelně můžeš přidat .env:
# EnvironmentFile=-/opt/projects/robotour/pilot/.env

ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/pilot/main.py

StandardOutput=append:/robot/data/logs/pilot/pilot.log
StandardError=append:/robot/data/logs/pilot/pilot.log

Restart=always
RestartSec=3
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-pilot.service

echo "✅ Service robot-pilot is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
