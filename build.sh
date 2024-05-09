export STATIC_ROSIDL_TYPESUPPORT_C=rosidl_typesupport_fastrtps_c
export STATIC_ROSIDL_TYPESUPPORT_CPP=rosidl_typesupport_fastrtps_cpp
WALI_DIR=/home/tc/WALI
WALI_LLVM_BIN=$WALI_DIR/llvm-project/build/bin

WALI_CC=$WALI_LLVM_BIN/clang
WALI_CXX=$WALI_LLVM_BIN/clang++
CFLAGS="-fPIC --target=wasm32-wasi-threads -pthread --sysroot=/$WALI_DIR/wali-musl/sysroot -matomics -mbulk-memory -mmutable-globals -msign-ext"
CXXFLAGS="-fPIC -stdlib=libc++ --target=wasm32-wasi-threads -pthread --sysroot=/$WALI_DIR/wali-musl/sysroot -I/$WALI_DIR/libcxx/include/c++/v1 -matomics -mbulk-memory -mmutable-globals -msign-ext"
LFLAGS="-L/$WALI_DIR/wali-musl/sysroot/lib -L/$WALI_DIR/libcxx/lib -Wl,--shared-memory -Wl,--export-memory -Wl,--max-memory=2147483648"


IGNORED_PACK="performance_test_fixture ament_cmake_google_benchmark rmw_cyclonedds_cpp osrf_testing_tools_cpp"
BUILD_PACK=examples_rclcpp_minimal_publisher

colcon graph --packages-up-to $BUILD_PACK --packages-ignore $IGNORED_PACK &> lastbuild.graph

colcon build --executor sequential --cmake-args  "-DCMAKE_BUILD_TYPE=Debug" "-DCMAKE_MESSAGE_LOG_LEVEL=STATUS" "-DCMAKE_VERBOSE_MAKEFILE=ON" \
         "-DRMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION=ON" "-DFORCE_BUILD_VENDOR_PKG=OFF" \
         "-DCMAKE_C_COMPILER=$WALI_CC" "-DCMAKE_CXX_COMPILER=$WALI_CXX"  \
         "-DCMAKE_C_FLAGS=$CFLAGS $LFLAGS" \
         "-DCMAKE_CXX_FLAGS=$CXXFLAGS $LFLAGS" \
         "-DCMAKE_LINKER=wasm-ld" \
         "-DCMAKE_EXE_LINKER_FLAGS=$LFLAGS" \
         "-DCMAKE_AR=$WALI_DIR/llvm-project/build/bin/llvm-ar" \
         "-DCMAKE_RANLIB=$WALI_DIR/llvm-project/build/bin/llvm-ranlib" \
         --symlink-install --event-handler console_direct+  \
         --packages-up-to $BUILD_PACK \
         --packages-ignore $IGNORED_PACK
source ./install/local_setup.bash

