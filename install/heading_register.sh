#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-heading.service"
LOG_DIR="/data/logs/heading"
LOG_FILE="$LOG_DIR/heading.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"

echo "🛠  Creating systemd service robot-heading"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot Heading for Robotour
Wants=network-online.target
After=network-online.target
# Volitelně lze zapnout závislosti na jiných službách:
# Wants=robot-gnss.service robot-pointperfect.service
# After=robot-gnss.service robot-pointperfect.service

# Pomůže zachytit chybějící soubory srozumitelněji než CHDIR fail
ConditionPathExists=/opt/projects/robotour/heading/main.py

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/heading

# před spuštěním ukonči libovolný proces, který drží port 9010
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9010/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
# Volitelně můžeš přidat .env:
# EnvironmentFile=-/opt/projects/robotour/heading/.env

ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/heading/main.py

StandardOutput=append:/data/logs/heading/heading.log
StandardError=append:/data/logs/heading/heading.log

Restart=always
RestartSec=3
NoNewPrivileges=true

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-heading.service

echo "✅ Service robot-heading is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
