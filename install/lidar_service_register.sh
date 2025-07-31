#!/bin/bash

SERVICE_FILE="/etc/systemd/system/robot-lidar.service"
LOG_DIR="/data/logs/lidar"
LOG_FILE="$LOG_DIR/lidar.log"

echo "📁 Vytvářím logovací složku..."
mkdir -p "$LOG_DIR"
touch "$LOG_FILE"
chmod 664 "$LOG_FILE"

echo "🛠️ Vytvářím systemd službu: robot-lidar.service"

sudo tee "$SERVICE_FILE" > /dev/null <<EOF
[Unit]
Description=Robotour 2025 – lidar socket server
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/server

# vlastní spuštění (un-buffer mód kvůli okamžitému logování)
# Environment=PYTHONUNBUFFERED=1

# před spuštěním ukonči libovolný proces, který drží port 9002
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9002/tcp || true'
ExecStartPre=/bin/sleep 0.5

ExecStart=/opt/projects/robotour/server/robot_lidar_tcp

# logujeme přes systemd přesměrování
StandardOutput=append:/data/logs/lidar/lidar.log
StandardError=append:/data/logs/lidar/lidar.log

Restart=always
RestartSec=2

[Install]
WantedBy=multi-user.target
EOF

echo "🔁 Aktivuji službu robot-lidar.service"
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-lidar.service
echo "   tail -f $LOG_FILE"