echo "Stopping and disabling Zeroconf service..."
sudo systemctl stop zeroconf.service
sudo systemctl disable zeroconf.service
sudo rm -f /etc/systemd/system/zeroconf.service
