# Copyright (C) 2018 deeplearningrobotics.ai
add_library(argos_ros_bot MODULE tensor_swarm_bot.h tensor_swar_bot.cpp)
add_library(argos_CLARE_ros_bot MODULE tensor_swarm_CLARE_bot.h tensor_swarm_CLARE_bot.cpp)

link_directories("/usr/local/lib/argos3")
include_directories(${CMAKE_SOURCE_DIR}/tensorswarm/plugin/Thesis_ARGoS/implementation_and_examples)


target_link_libraries(argos_ros_bot
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  agent_implementation
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

target_link_libraries(argos_CLARE_ros_bot
  argos3core_simulator
  argos3plugin_simulator_footbot
  argos3plugin_simulator_genericrobot
  agent_implementation
  /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

add_library(argos_loop_function MODULE
    ros_service_loop_function.cpp ros_service_loop_function.h
    tensor_swarm_bot.h tensor_swar_bot.cpp
    ros_helpers.h)
target_link_libraries(argos_loop_function
    argos3core_simulator
    argos3plugin_simulator_footbot
    argos3plugin_simulator_genericrobot
    /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

add_library(argos_loop_function_CLARE MODULE
        ros_service_loop_function_CLARE.cpp ros_service_loop_function_CLARE.h
        tensor_swarm_CLARE_bot.h tensor_swarm_CLARE_bot.cpp
        ros_helpers.h)
target_link_libraries(argos_loop_function_CLARE
        argos3core_simulator
        argos3plugin_simulator_footbot
        argos3plugin_simulator_genericrobot
        argos3plugin_simulator_pipuck
        /opt/ros/$ENV{ROS_DISTRO}/lib/libroscpp.so)

