# build sequence, result in bin
mkdir build
cd build
cmake ..
make -j$(nproc)