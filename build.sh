export STATIC_ROSIDL_TYPESUPPORT_C=rosidl_typesupport_fastrtps_c
export STATIC_ROSIDL_TYPESUPPORT_CPP=rosidl_typesupport_fastrtps_cpp
colcon build --cmake-args "-DCMAKE_C_COMPILER=clang" "-DCMAKE_CXX_COMPILER=clang++" "-DCMAKE_BUILD_TYPE=Debug" "-DCMAKE_MESSAGE_LOG_LEVEL=STATUS" "-DCMAKE_C_FLAGS=-fPIC" "-DCMAKE_CXX_FLAGS=-fPIC" "-DRMW_IMPLEMENTATION_DISABLE_RUNTIME_SELECTION=ON" --packages-up-to demo_nodes_cpp --symlink-install --event-handler console_direct+
