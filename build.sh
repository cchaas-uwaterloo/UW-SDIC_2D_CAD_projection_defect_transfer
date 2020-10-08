TARGET=${FILENAME}

[ -d build ] || { mkdir build; [ -d build ] || exit 1; }
cd build || exit 1
rm intstall/bin/cam_cad_proj
cmake .. -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_TOOLCHAIN_FILE=/home/cameron/vcpkg/scripts/buildsystems/vcpkg.cmake
make 
make install

./cam_cad_proj