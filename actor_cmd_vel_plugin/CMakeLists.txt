cmake_minimum_required(VERSION 3.8)
project(actor_cmd_vel_plugin)

set(CMAKE_CXX_STANDARD 17)


find_package(gz-sim8 REQUIRED)
find_package(gz-plugin2 REQUIRED COMPONENTS register)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)


add_library(ActorCmdVelPlugin SHARED
  src/ActorCmdVelPlugin.cpp
)

target_include_directories(ActorCmdVelPlugin PRIVATE
  ${rclcpp_INCLUDE_DIRS}
)

ament_target_dependencies(ActorCmdVelPlugin
  rclcpp
  geometry_msgs
)

target_link_libraries(ActorCmdVelPlugin
  gz-sim8::core
  gz-plugin2::gz-plugin2
)

install(TARGETS ActorCmdVelPlugin
  LIBRARY DESTINATION lib
)

ament_package()
