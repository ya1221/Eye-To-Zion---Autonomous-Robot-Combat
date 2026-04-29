cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)

# 1. Build the Hardware Plugin Library
add_library(motor_driver SHARED
  src/motor_driver.cpp
)

# 2. Add include directories
target_include_directories(motor_driver PRIVATE include)

# 3. Add dependencies to the library
ament_target_dependencies(motor_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# 4. Register the plugin (very important!)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugins.xml)

# 5. Install the library
install(TARGETS motor_driver
  DESTINATION lib
)

# 6. Install folders (config, launch) so get_package_share_directory can find them
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
