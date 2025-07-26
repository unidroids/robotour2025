# logy
mkdir -p /data/logs/fastapi
chown user:user /data/logs/fastapi
chmod 755 /data/logs/fastapi

touch /data/logs/fastapi/fastapi.log
chmod 664 /data/logs/fastapi/fastapi.log

# vypis
nano /data/logs/fastapi/fastapi.log 

# installace
chmod +x register_fastapi.sh unregister_fastapi.sh
./register_fastapi.sh
systemctl status fastapi-server

# uninstallace
./unregister_fastapi.sh 



