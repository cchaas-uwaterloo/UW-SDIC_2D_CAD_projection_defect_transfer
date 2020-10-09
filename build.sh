TARGET=${FILENAME}

[ -d build ] || { mkdir build; [ -d build ] || exit 1; }
cd build || exit 1
cmake .. 
make 
