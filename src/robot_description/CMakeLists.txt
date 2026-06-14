cmake_minimum_required(VERSION 3.8)
project(robot_description)

if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ----------------------- #
# --- 1. Dependencies --- #
# ----------------------- #

# Crucial: Tell CMake to look in /usr/local where you installed the library
list(APPEND CMAKE_PREFIX_PATH "/usr/local")

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(CURL REQUIRED)
find_package(InfluxDB REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)
find_package(fmt REQUIRED)
find_package(OpenCV REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(rclcpp_components REQUIRED)

# ---------------------------- #
# --- 2. Executables Nodes --- #
# ---------------------------- #

# --- TwistToAckermann node --- #
add_executable(TwistToAckermann src/TwistToAckermann.cpp)

target_include_directories(TwistToAckermann PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(TwistToAckermann PUBLIC cxx_std_17)

ament_target_dependencies(TwistToAckermann 
  rclcpp
  geometry_msgs
)

# --- telegraf_bridge node --- #
add_executable(telegraf_bridge_node 
  src/telegraf_bridge.cpp
  src/telemetry_sender.cpp  
)

target_include_directories(telegraf_bridge_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(telegraf_bridge_node PUBLIC cxx_std_17)

ament_target_dependencies(telegraf_bridge_node 
  rclcpp
  nav_msgs
)

target_link_libraries(telegraf_bridge_node
  fmt::fmt
  Boost::headers
  Boost::system   
)

# --- map_to_image node --- #
add_executable(map_to_image_node src/map_to_image_node.cpp)

target_include_directories(map_to_image_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(map_to_image_node PUBLIC cxx_std_17)

ament_target_dependencies(map_to_image_node 
  rclcpp
  nav_msgs
)

target_include_directories(map_to_image_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(map_to_image_node ${OpenCV_LIBS})

# --- gstreamer_sender node --- #
add_executable(gstreamer_sender_node src/gstreamer_sender.cpp)

target_include_directories(gstreamer_sender_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)

target_compile_features(gstreamer_sender_node PUBLIC cxx_std_17)

ament_target_dependencies(
  gstreamer_sender_node 
  rclcpp
  sensor_msgs
  cv_bridge
)

target_include_directories(gstreamer_sender_node PUBLIC ${OpenCV_INCLUDE_DIRS})
target_link_libraries(gstreamer_sender_node ${OpenCV_LIBS})

# ------------------------ #
# --- 3. Install Rules --- #
# ------------------------ #

# install(TARGETS twist_to_ackermann_component
#   ARCHIVE DESTINATION lib
#   LIBRARY DESTINATION lib
#   RUNTIME DESTINATION bin
# )

install(TARGETS 
  TwistToAckermann
  telegraf_bridge_node
  map_to_image_node
  gstreamer_sender_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch/ DESTINATION share/${PROJECT_NAME}/launch)
install(DIRECTORY world/  DESTINATION share/${PROJECT_NAME}/world)
install(DIRECTORY config/ DESTINATION share/${PROJECT_NAME}/config)
install(DIRECTORY rviz2/  DESTINATION share/${PROJECT_NAME}/rviz2)
install(DIRECTORY include/ DESTINATION include)

ament_package()