cmake_minimum_required(VERSION 2.8.3)
project(inertial_sense_ros)

find_package(yaml-cpp REQUIRED)
find_package(ament_cmake REQUIRED)

set(dependencies
  inertial_sense_msgs
  rclcpp
  rclpy
  std_msgs
  sensor_msgs
  geometry_msgs
  diagnostic_msgs
  tf2_ros
)

find_package(${dependencies} REQUIRED)
find_package(Threads)

SET(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -std=gnu11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX" )
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=gnu++11 -fms-extensions -Wl,--no-as-needed -DPLATFORM_IS_LINUX")



include_directories(include
  ${YAML_CPP_INCLUDE_DIR}
  lib/inertial-sense-sdk/src #This line of CMakeList.txt stays in .external file to reference submodule
)

add_subdirectory(lib/inertial-sense-sdk)

add_library(inertial_sense_ros
        src/inertial_sense_ros.cpp
)
target_link_libraries(inertial_sense_ros InertialSense ${YAML_CPP_LIBRARIES} pthread)
target_include_directories(inertial_sense_ros PUBLIC include lib/inertial-sense-sdk/src)
add_dependencies(inertial_sense_ros inertial_sense_ros_generate_messages_cpp)

add_executable(inertial_sense_node src/inertial_sense_node.cpp)
target_link_libraries(inertial_sense_node inertial_sense_ros )


ament_package()
# catkin_add_gtest(test_client_reconnect test/test_client_reconnect.cpp)
# target_link_libraries(test_client_reconnect ${PROJECT_NAME})

