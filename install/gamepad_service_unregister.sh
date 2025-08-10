#!/bin/bash
set -euo pipefail
SERVICE_FILE="/etc/systemd/system/robot-gamepad.service"

if systemctl is-enabled --quiet robot-gamepad.service; then
  sudo systemctl disable --now robot-gamepad.service || true
fi

sudo rm -f "$SERVICE_FILE"
sudo systemctl daemon-reload

echo "✅ robot-gamepad odstraněn"