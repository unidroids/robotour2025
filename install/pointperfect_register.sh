#!/bin/bash
set -e

SERVICE_PATH="/etc/systemd/system/robot-pointperfect.service"
LOG_DIR="/robot/data/logs/pointperfect"
LOG_FILE="$LOG_DIR/pointperfect.log"
FULL_LOG_FILE="$LOG_DIR/pointperfect_full.log"

echo "📁 Creating log directory..."
sudo mkdir -p "$LOG_DIR"
sudo touch "$LOG_FILE"
sudo chmod 664 "$LOG_FILE"
sudo touch "$FULL_LOG_FILE"
sudo chmod 664 "$FULL_LOG_FILE"
sudo chown -R user:user "$LOG_DIR"

echo "🛠  Creating systemd service robot-pointperfect"

sudo tee "$SERVICE_PATH" > /dev/null <<'EOF'
[Unit]
Description=Robot PointPerfect client for Robotour
After=network.target

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/pointperfect

# před spuštěním ukonči libovolný proces, který drží port 9007
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9007/tcp || true'
ExecStartPre=/bin/sleep 0.5

Environment=PYTHONUNBUFFERED=1
# případné proměnné pro klíče můžeš doplnit sem, např.:
# Environment=PP_API_KEY=xxx PP_TOPIC=yyy
ExecStart=/robot/opt/projects/robotour/venv-robotour/bin/python /opt/projects/robotour/pointperfect/main.py

StandardOutput=append:/robot/data/logs/pointperfect/pointperfect.log
StandardError=append:/robot/data/logs/pointperfect/pointperfect.log

Restart=always
RestartSec=3

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Reloading and enabling service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now robot-pointperfect.service

echo "✅ Service robot-pointperfect is now active. Check logs with:"
echo "   tail -f $LOG_FILE"
