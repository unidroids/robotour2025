#!/bin/bash

SERVICE_FILE="/etc/systemd/system/journey.service"
LOG_DIR="/data/logs/journey"
LOG_FILE="$LOG_DIR/journey.log"

echo "📁 Vytvářím logovací složku..."
mkdir -p "$LOG_DIR"
touch "$LOG_FILE"
chmod 664 "$LOG_FILE"

echo "🛠️ Vytvářím systemd službu: journey.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 – workflow orchestrátor (Journey)
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/journey

# Ne-bufferovaný výstup pro okamžité logování
Environment=PYTHONUNBUFFERED=1

# před spuštěním ukonči libovolný proces, který drží port 9004
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9004/tcp || true'
ExecStartPre=/bin/sleep 0.5

ExecStart=/usr/bin/python3 main.py

# Logování přes systemd
StandardOutput=append:/data/logs/journey/journey.log
StandardError=append:/data/logs/journey/journey.log

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "🔁 Aktivuji službu journey.service"
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now journey.service
echo "   tail -f $LOG_FILE"