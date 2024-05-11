#!/bin/bash

IGNORED_PACK="performance_test_fixture ament_cmake_google_benchmark osrf_testing_tools_cpp"
IGNORED_REGEX="rmw_connextdds* rmw_fastrtps* iceoryx*"
BUILD_PACK=examples_rclcpp_minimal_publisher

colcon graph --packages-up-to $BUILD_PACK --packages-ignore $IGNORED_PACK --packages-ignore-regex $IGNORE_REGEX &> lastbuild.graph
