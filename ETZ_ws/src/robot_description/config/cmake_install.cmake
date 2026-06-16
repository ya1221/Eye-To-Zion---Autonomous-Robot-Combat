# Install script for directory: /home/itay3711/AutonomousWarfare/src/robot_description

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/TwistToAckermann" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/TwistToAckermann")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/TwistToAckermann"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_description" TYPE EXECUTABLE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/TwistToAckermann")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/TwistToAckermann" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/TwistToAckermann")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/TwistToAckermann"
         OLD_RPATH "/home/itay3711/ros2_humble/install/rclcpp/lib:/home/itay3711/ros2_humble/install/geometry_msgs/lib:/home/itay3711/ros2_humble/install/libstatistics_collector/lib:/home/itay3711/ros2_humble/install/rcl/lib:/home/itay3711/ros2_humble/install/rmw_implementation/lib:/home/itay3711/ros2_humble/install/ament_index_cpp/lib:/home/itay3711/ros2_humble/install/rcl_logging_spdlog/lib:/home/itay3711/ros2_humble/install/rcl_logging_interface/lib:/home/itay3711/ros2_humble/install/rcl_interfaces/lib:/home/itay3711/ros2_humble/install/rcl_yaml_param_parser/lib:/home/itay3711/ros2_humble/install/libyaml_vendor/lib:/home/itay3711/ros2_humble/install/rosgraph_msgs/lib:/home/itay3711/ros2_humble/install/statistics_msgs/lib:/home/itay3711/ros2_humble/install/tracetools/lib:/home/itay3711/ros2_humble/install/std_msgs/lib:/home/itay3711/ros2_humble/install/builtin_interfaces/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/itay3711/ros2_humble/install/fastcdr/lib:/home/itay3711/ros2_humble/install/rmw/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_c/lib:/home/itay3711/ros2_humble/install/rcpputils/lib:/home/itay3711/ros2_humble/install/rosidl_runtime_c/lib:/home/itay3711/ros2_humble/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/TwistToAckermann")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/sensor_data_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/sensor_data_node")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/sensor_data_node"
         RPATH "")
  endif()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib/robot_description" TYPE EXECUTABLE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/sensor_data_node")
  if(EXISTS "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/sensor_data_node" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/sensor_data_node")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/sensor_data_node"
         OLD_RPATH "/home/itay3711/ros2_humble/install/rclcpp/lib:/home/itay3711/ros2_humble/install/nav_msgs/lib:/home/itay3711/ros2_humble/install/libstatistics_collector/lib:/home/itay3711/ros2_humble/install/rcl/lib:/home/itay3711/ros2_humble/install/rmw_implementation/lib:/home/itay3711/ros2_humble/install/ament_index_cpp/lib:/home/itay3711/ros2_humble/install/rcl_logging_spdlog/lib:/home/itay3711/ros2_humble/install/rcl_logging_interface/lib:/home/itay3711/ros2_humble/install/rcl_interfaces/lib:/home/itay3711/ros2_humble/install/rcl_yaml_param_parser/lib:/home/itay3711/ros2_humble/install/libyaml_vendor/lib:/home/itay3711/ros2_humble/install/rosgraph_msgs/lib:/home/itay3711/ros2_humble/install/statistics_msgs/lib:/home/itay3711/ros2_humble/install/tracetools/lib:/home/itay3711/ros2_humble/install/geometry_msgs/lib:/home/itay3711/ros2_humble/install/std_msgs/lib:/home/itay3711/ros2_humble/install/builtin_interfaces/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_fastrtps_c/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_fastrtps_cpp/lib:/home/itay3711/ros2_humble/install/fastcdr/lib:/home/itay3711/ros2_humble/install/rmw/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_introspection_cpp/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_introspection_c/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_cpp/lib:/home/itay3711/ros2_humble/install/rosidl_typesupport_c/lib:/home/itay3711/ros2_humble/install/rcpputils/lib:/home/itay3711/ros2_humble/install/rosidl_runtime_c/lib:/home/itay3711/ros2_humble/install/rcutils/lib:"
         NEW_RPATH "")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/robot_description/sensor_data_node")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description/launch" TYPE DIRECTORY FILES "/home/itay3711/AutonomousWarfare/src/robot_description/launch/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description/world" TYPE DIRECTORY FILES "/home/itay3711/AutonomousWarfare/src/robot_description/world/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description/config" TYPE DIRECTORY FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/itay3711/AutonomousWarfare/src/robot_description/include/")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/package_run_dependencies" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_index/share/ament_index/resource_index/package_run_dependencies/robot_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/parent_prefix_path" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_index/share/ament_index/resource_index/parent_prefix_path/robot_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description/environment" TYPE FILE FILES "/home/itay3711/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/ament_prefix_path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description/environment" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_environment_hooks/ament_prefix_path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description/environment" TYPE FILE FILES "/home/itay3711/ros2_humble/install/ament_cmake_core/share/ament_cmake_core/cmake/environment_hooks/environment/path.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description/environment" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_environment_hooks/path.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_environment_hooks/local_setup.bash")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_environment_hooks/local_setup.sh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_environment_hooks/local_setup.zsh")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_environment_hooks/local_setup.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_environment_hooks/package.dsv")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/ament_index/resource_index/packages" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_index/share/ament_index/resource_index/packages/robot_description")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description/cmake" TYPE FILE FILES
    "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_core/robot_descriptionConfig.cmake"
    "/home/itay3711/AutonomousWarfare/src/robot_description/config/ament_cmake_core/robot_descriptionConfig-version.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/share/robot_description" TYPE FILE FILES "/home/itay3711/AutonomousWarfare/src/robot_description/package.xml")
endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/itay3711/AutonomousWarfare/src/robot_description/config/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
