#!/bin/bash

IGNORED_PACK="performance_test_fixture ament_cmake_google_benchmark"
BUILD_PACK=examples_rclcpp_minimal_publisher

colcon graph --packages-up-to $BUILD_PACK --packages-ignore $IGNORED_PACK &> lastbuild.graph
