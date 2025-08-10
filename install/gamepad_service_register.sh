#!/bin/bash
set -euo pipefail
SERVICE_FILE="/etc/systemd/system/robot-gamepad.service"
LOG_DIR="/data/logs/gamepad"
VENV_BIN="/robot/opt/projects/robotour/venv-robotour/bin"
WORK_DIR="/opt/projects/robotour/gamepad"

echo "📁 Vytvářím logovací složku..."
sudo mkdir -p "$LOG_DIR"
sudo chown -R user:user "$LOG_DIR"

# Přístup k /dev/input/js*, většinou skupina input
echo "👤 Přidávám uživatele 'user' do skupiny 'input' (odhlásit/přihlásit po prvním vytvoření)."
sudo usermod -aG input user || true

# Závislosti ve venv: pygame
echo "📦 Instalace pygame do venv (pokud chybí)"
sudo -u user "$VENV_BIN/pip" install --upgrade pip >/dev/null || true
sudo -u user "$VENV_BIN/pip" show pygame >/dev/null 2>&1 || sudo -u user "$VENV_BIN/pip" install pygame

cat <<EOF | sudo tee "$SERVICE_FILE" > /dev/null
[Unit]
Description=Robotour 2025 – server-gamepad (TCP 9005)
After=network.target

[Service]
User=user
WorkingDirectory=$WORK_DIR
Environment=PYTHONUNBUFFERED=1
ExecStartPre=/bin/bash -c '/usr/bin/fuser -k 9005/tcp || true'
ExecStartPre=/bin/sleep 0.5
ExecStart=$VENV_BIN/python $WORK_DIR/gamepad_server.py
StandardOutput=append:$LOG_DIR/service.log
StandardError=append:$LOG_DIR/service.log
Restart=always
RestartSec=1

[Install]
WantedBy=multi-user.target
EOF

echo "🔄 Načítám systemd..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
echo "▶️ Povoluji a spouštím službu..."
sudo systemctl enable --now robot-gamepad.service

sleep 0.3
sudo systemctl --no-pager --full status robot-gamepad.service || true

echo "   tail -f $LOG_DIR/service.log"