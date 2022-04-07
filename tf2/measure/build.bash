cd ../cmake-build-release || exit
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j$(nproc)
cd - || exit