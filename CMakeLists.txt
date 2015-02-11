cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

set(ROS_BUILD_TYPE Release)

rosbuild_init()

set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

rosbuild_add_boost_directories()

rosbuild_add_library(explore_multirobot_lib src/explore_frontier.cpp src/loop_closure.cpp src/multi_robot.cpp)
set_target_properties(explore_multirobot_lib PROPERTIES OUTPUT_NAME "explore_multirobot")
rosbuild_link_boost(explore_multirobot_lib thread)

rosbuild_add_executable(explore_multirobot src/explore.cpp)
target_link_libraries(explore_multirobot explore_multirobot_lib)
