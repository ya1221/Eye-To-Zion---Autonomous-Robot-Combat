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
find_package(tf2 REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(PkgConfig REQUIRED)
pkg_check_modules(GPIOD REQUIRED libgpiod)

# -------------------- #
# --- motor_driver --- #
# -------------------- #

# Build the Hardware Plugin Library
add_library(motor_driver SHARED
  src/motor_driver.cpp
)

# Add include directories
target_include_directories(motor_driver PRIVATE include ${GPIOD_INCLUDE_DIRS})

# Add dependencies to the library
ament_target_dependencies(motor_driver
  hardware_interface
  pluginlib
  rclcpp
  rclcpp_lifecycle
)

# Link libgpiod
target_link_libraries(motor_driver ${GPIOD_LIBRARIES})

# Register the plugin (very important!)
pluginlib_export_plugin_description_file(hardware_interface hardware_interface_plugins.xml)


install(TARGETS motor_driver
  DESTINATION lib
)


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

install(TARGETS twist_to_ackermann
  DESTINATION lib/${PROJECT_NAME}
)

# -----------------------------
# --- heading_pid_controller --
# -----------------------------
add_executable(heading_pid_controller src/pid_controller.cpp)

target_include_directories(heading_pid_controller PRIVATE
  include
)

ament_target_dependencies(
  heading_pid_controller
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  tf2
)

install(TARGETS heading_pid_controller
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)



# --------------- #
# --- Install --- #
# --------------- #

# Install folders (config, launch) so get_package_share_directory can find them
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
