# logy
mkdir -p /data/logs/fastapi
chown user:user /data/logs/fastapi
chmod 755 /data/logs/fastapi

touch /data/logs/fastapi/fastapi.log
chmod 664 /data/logs/fastapi/fastapi.log

touch /data/logs/fastapi/zeroconf.log
chmod 664 /data/logs/fastapi/zeroconf.log

# installace
chmod +x register_fastapi.sh unregister_fastapi.sh
./register_fastapi.sh
systemctl status fastapi-server

chmod +x register_zeroconf.sh unregister_zeroconf.sh
./register_zeroconf.sh
systemctl status zeroconf


# uninstallace
./unregister_fastapi.sh 
./unregister_zeroconf.sh 

# status
systemctl status fastapi-server
systemctl status zeroconf


# vypis
nano /data/logs/fastapi/fastapi.log 
tail -f /data/logs/fastapi/fastapi.log

nano /data/logs/fastapi/zeroconf.log
tail -f /data/logs/fastapi/zeroconf.log