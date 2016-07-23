echo "building libviso2..."

cd ThirdParty/libviso2
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

echo "building calibration of stereo-odometry tool..."
cd ../../../
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
