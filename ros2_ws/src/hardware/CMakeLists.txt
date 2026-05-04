cmake_minimum_required(VERSION 3.8)
project(hardware)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ackermann_steering_controller REQUIRED)
find_package(joint_state_broadcaster REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(geometry_msgs REQUIRED)



# -------------------- #
# --- motor_driver --- #
# -------------------- #

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




# -------------------------- #
# --- twist_to_ackermann --- #
# -------------------------- #


# Executable 1: Convert Twist to Ackermann
add_executable(twist_to_ackermann
 src/twist_to_ackermann.cpp
)

target_include_directories(twist_to_ackermann PRIVATE
  include
)

ament_target_dependencies(twist_to_ackermann 
  rclcpp
  geometry_msgs
)

# --------------- #
# --- Install --- #
# --------------- #

# 5. Install the library
install(TARGETS motor_driver
  DESTINATION lib
)

install(TARGETS twist_to_ackermann
  DESTINATION lib/${PROJECT_NAME}
)

# 6. Install folders (config, launch) so get_package_share_directory can find them
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

install(FILES ros2_control.xacro hardware_interface_plugins.xml 
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
