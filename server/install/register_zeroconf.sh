ZEROCONF_SERVICE="/etc/systemd/system/zeroconf.service"
ZEROCONF_LOG="/data/logs/fastapi/zeroconf.log"

echo "Creating log file for zeroconf..."
mkdir -p /data/logs/fastapi
touch $ZEROCONF_LOG
chmod 664 $ZEROCONF_LOG

echo "Creating Zeroconf systemd service..."

sudo tee $ZEROCONF_SERVICE > /dev/null <<EOF
[Unit]
Description=Zeroconf (Bonjour) publisher for FastAPI
After=network.target
Requires=fastapi-server.service

[Service]
User=user
WorkingDirectory=/opt/projects/robotour/server
ExecStart=/bin/bash -c '/opt/projects/robotour/venv-robotour/bin/python bojour.py >> $ZEROCONF_LOG 2>&1'
Restart=always
RestartSec=2
StandardOutput=append:$ZEROCONF_LOG
StandardError=append:$ZEROCONF_LOG

[Install]
WantedBy=multi-user.target
EOF

echo "Enabling Zeroconf service..."
sudo systemctl daemon-reexec
sudo systemctl daemon-reload
sudo systemctl enable --now zeroconf.service
