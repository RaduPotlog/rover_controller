cmake_minimum_required(VERSION 3.8)
project(rover_c1_hardware)

if(CMAKE_CXX_COMPILER_ID MATCHES "(GNU|Clang)")
  add_compile_options(-Wall -Wextra -Werror=conversion -Werror=unused-but-set-variable -Werror=return-type -Werror=shadow)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(control_toolbox REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)


find_package(ardupilot_msgs REQUIRED)

add_library(
  rover_c1_hardware
  SHARED
  src/rover_c1_system_hardware_if.cpp
  include/rover_c1_hardware/rover_c1_system_hardware_if.hpp
)

target_include_directories(
  rover_c1_hardware
  PUBLIC
  include
)
ament_target_dependencies(
  rover_c1_hardware
  hardware_interface
  control_toolbox
  rclcpp
  rclcpp_lifecycle
  ardupilot_msgs
)
# prevent pluginlib from using boost
target_compile_definitions(rover_c1_hardware PUBLIC "PLUGINLIB__DISABLE_BOOST_FUNCTIONS")

pluginlib_export_plugin_description_file(
  hardware_interface rover_c1_hardware.xml)

install(
  TARGETS
  rover_c1_hardware
  RUNTIME DESTINATION bin
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(
  include
)

ament_export_libraries(
  rover_c1_hardware
)

ament_export_dependencies(
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()
