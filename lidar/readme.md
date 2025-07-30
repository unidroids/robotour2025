# build sequence, result in bin
mkdir build
cd build
cmake ..
make -j$(nproc)


chmod +x ../bin/robot_lidar_tcp
cp ../bin/robot_lidar_tcp ../../server/robot_lidar_tcp
