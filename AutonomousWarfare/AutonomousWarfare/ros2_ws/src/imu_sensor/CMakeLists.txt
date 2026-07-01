cmake_minimum_required(VERSION 3.8)
project(imu_sensor)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)

# ─── Executable ────────────────────────────────────────────────────────────
add_executable(imu_sensor_driver_node
  src/imu_sensor_driver.cpp
)

target_include_directories(imu_sensor_driver_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(imu_sensor_driver_node
  rclcpp
  sensor_msgs
  std_srvs
)
target_link_libraries(imu_sensor_driver_node i2c)

target_compile_features(imu_sensor_driver_node PUBLIC c_std_99 cxx_std_17)

# ─── Install ───────────────────────────────────────────────────────────────
install(TARGETS imu_sensor_driver_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)

install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
