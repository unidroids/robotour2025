#!/bin/bash

SERVICE_FILE="/etc/systemd/system/robot-cameras.service"
LOG_DIR="/data/logs/camera"
LOG_FILE="$LOG_DIR/cameras.log"

echo "📁 Vytvářím logovací složku..."
mkdir -p "$LOG_DIR"
touch "$LOG_FILE"
chmod 664 "$LOG_FILE"

echo "🛠️ Vytvářím systemd službu: robot-cameras.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 – kamera socket server
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/server

# vlastní spuštění (un-buffer mód kvůli okamžitému logování)
Environment=PYTHONUNBUFFERED=1

# před spuštěním ukonči libovolný proces, který drží port 9001
# ExecStartPre=/usr/bin/fuser -k 9001/tcp
ExecStartPre=-/usr/bin/fuser -k 9001/tcp
ExecStartPre=/bin/sleep 0.5

ExecStart=/usr/bin/python3 cameras.py

# logujeme přes systemd přesměrování
StandardOutput=append:/data/logs/camera/cameras.log
StandardError=append:/data/logs/camera/cameras.log

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "🔁 Aktivuji službu robot-cameras.service"
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-cameras.service
echo "   tail -f $LOG_FILE"
