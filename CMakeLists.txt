cmake_minimum_required(VERSION 3.16)
project(astra_slam LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
add_compile_options(-Wall -Wextra -Wpedantic)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(OpenGL REQUIRED)
find_package(GLEW REQUIRED)
find_package(glfw3 REQUIRED)

add_library(astra_slam_core
  src/occupancy_grid.cpp
  src/scan_matcher.cpp
)
target_include_directories(astra_slam_core PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(sensor_sim_node src/sensor_sim_node.cpp)
target_link_libraries(sensor_sim_node astra_slam_core)
ament_target_dependencies(sensor_sim_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_ros)

add_executable(fusion_slam_node src/fusion_slam_node.cpp)
target_link_libraries(fusion_slam_node astra_slam_core)
ament_target_dependencies(fusion_slam_node
  rclcpp sensor_msgs nav_msgs geometry_msgs visualization_msgs
  tf2 tf2_geometry_msgs)

add_executable(opengl_viewer_node src/opengl_viewer_node.cpp)
target_link_libraries(opengl_viewer_node
  astra_slam_core OpenGL::GL GLEW::GLEW glfw)
ament_target_dependencies(opengl_viewer_node
  rclcpp sensor_msgs nav_msgs geometry_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  astra_slam_core sensor_sim_node fusion_slam_node opengl_viewer_node
  EXPORT export_astra_slam
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch config DESTINATION share/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  ament_add_gtest(test_core test/test_core.cpp)
  target_link_libraries(test_core astra_slam_core)
  target_include_directories(test_core PRIVATE include)
endif()

ament_export_include_directories(include)
ament_export_libraries(astra_slam_core)
ament_package()
