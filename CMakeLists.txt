cmake_minimum_required(VERSION 3.8)
project(cobot_hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(lifecycle_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

add_library(cobot_hardware SHARED src/cobot_hardware.cpp)

target_compile_features(cobot_hardware PUBLIC cxx_std_17)
target_include_directories(cobot_hardware PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cobot_hardware
  hardware_interface
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface cobot_hardware_plugins.xml)

# Install
install(
  DIRECTORY include/
  DESTINATION include
)

install(TARGETS cobot_hardware
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_targets(
  export_${PROJECT_NAME}
  HAS_LIBRARY_TARGET
)
ament_export_dependencies(
  hardware_interface
  lifecycle_msgs
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

ament_package()