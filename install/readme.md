# logy
mkdir -p /data/logs/fastapi
chown user:user /data/logs/fastapi
chmod 755 /data/logs/fastapi

touch /data/logs/fastapi/fastapi.log
chmod 664 /data/logs/fastapi/fastapi.log

touch /data/logs/fastapi/zeroconf.log
chmod 664 /data/logs/fastapi/zeroconf.log

touch /data/logs/fastapi/cameras.log
chmod 664 /data/logs/fastapi/cameras.log

# installace
chmod +x register_fastapi.sh unregister_fastapi.sh
./register_fastapi.sh
systemctl status fastapi-server

chmod +x register_zeroconf.sh unregister_zeroconf.sh
./register_zeroconf.sh
systemctl status zeroconf

chmod +x register_camera_service.sh unregister_camera_service.sh
./register_camera_service.sh
systemctl status robot-cameras

chmod +x lidar_service_register.sh lidar_service_unregister.sh
./lidar_service_register.sh
systemctl status robot-lidar

 chmod +x gamepad_service_register.sh gamepad_service_unregister.sh 

# uninstallace
./unregister_fastapi.sh 
./unregister_zeroconf.sh 

# status
systemctl status fastapi-server
systemctl status zeroconf
systemctl status robot-cameras
systemctl status robot-lidar

# restart
sudo systemctl restart fastapi-server
sudo systemctl restart zeroconf
sudo systemctl restart robot-cameras
sudo systemctl restart robot-lidar
sudo systemctl restart robot-journey
sudo systemctl restart robot-gamepad
sudo systemctl restart robot-gnss

# vypis
nano /data/logs/fastapi/fastapi.log 
tail -f /data/logs/fastapi/fastapi.log

nano /data/logs/fastapi/zeroconf.log
tail -f /data/logs/fastapi/zeroconf.log

nano /data/logs/camera/cameras.log
tail -f /data/logs/camera/cameras.log

nano /data/logs/lidar/lidar.log
tail -f /data/logs/lidar/lidar.log

tail -f /data/logs/drive/drive.log