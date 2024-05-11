#!/bin/bash

export STATIC_ROSIDL_TYPESUPPORT_C=rosidl_typesupport_introspection_c
export STATIC_ROSIDL_TYPESUPPORT_CPP=rosidl_typesupport_introspection_cpp

source /home/tc/WALI/wali_bashenv.sh

IGNORED_PACK="performance_test_fixture ament_cmake_google_benchmark osrf_testing_tools_cpp mimick_vendor"
IGNORED_REGEX="rmw_connextdds* rmw_fastrtps* iceoryx* rosidl_typesupport_fastrtps*"
BUILD_PACK=examples_rclcpp_minimal_publisher

colcon graph --packages-up-to $BUILD_PACK --packages-ignore $IGNORED_PACK --packages-ignore-regex $IGNORED_REGEX &> lastbuild.graph

colcon build --cmake-args  "-DCMAKE_BUILD_TYPE=Debug" "-DCMAKE_MESSAGE_LOG_LEVEL=STATUS" "-DCMAKE_VERBOSE_MAKEFILE=ON" \
         "-DRMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION=ON" "-DFORCE_BUILD_VENDOR_PKG=OFF" "-DBUILD_SHARED_LIBS=OFF" "-DBUILD_TESTING=OFF" "-DBUILD_IDLC_TESTING=OFF" \
         "-DENABLE_SHM=OFF" "-DENABLE_SECURITY=OFF" "-DENABLE_SSL=OFF" "-DBUILD_IDLC=OFF" "-DBUILD_DDSPERF=OFF" "-DINSTALL_PDB=OFF" \
         "-DCMAKE_TOOLCHAIN_FILE=/home/tc/ros2_humble/toolchain.cmake" \
         --symlink-install --event-handler console_direct+  \
         --packages-up-to $BUILD_PACK \
         --packages-ignore $IGNORED_PACK \
         --packages-ignore-regex $IGNORED_REGEX \
source ./install/local_setup.bash

