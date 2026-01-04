cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(TwistToAckermann
  src/TwistToAckermann.cpp)

target_include_directories(TwistToAckermann PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(TwistToAckermann PUBLIC cxx_std_17)

ament_target_dependencies(TwistToAckermann  
  rclcpp
  geometry_msgs
)

install(TARGETS TwistToAckermann
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY world/  DESTINATION share/${PROJECT_NAME}/world)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY include/ DESTINATION include)

ament_package()
