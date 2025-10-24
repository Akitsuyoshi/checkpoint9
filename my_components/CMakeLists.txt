cmake_minimum_required(VERSION 3.8)
project(my_components)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(composition REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(custom_interfaces REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

include_directories(include)

add_library(preapproach_component SHARED src/preapproach_component.cpp)
add_library(attachserver_component SHARED src/attachserver_component.cpp)
add_library(attachclient_component SHARED src/attachclient_component.cpp)

target_compile_definitions(preapproach_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_compile_definitions(attachserver_component PRIVATE "COMPOSITION_BUILDING_DLL")
target_compile_definitions(attachclient_component PRIVATE "COMPOSITION_BUILDING_DLL")

ament_target_dependencies(preapproach_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2
  tf2_geometry_msgs
)
ament_target_dependencies(attachserver_component
  rclcpp
  rclcpp_components
  geometry_msgs
  sensor_msgs
  nav_msgs
  tf2_ros
  std_msgs
  custom_interfaces
)
ament_target_dependencies(attachclient_component
  rclcpp
  rclcpp_components
  custom_interfaces
)

rclcpp_components_register_nodes(preapproach_component "my_components::PreApproach")
rclcpp_components_register_nodes(attachserver_component "my_components::AttachServer")
rclcpp_components_register_nodes(attachclient_component "my_components::AttachClient")

set(node_plugins "${node_plugins}my_components::PreApproach;$<TARGET_FILE:preapproach_component>\n")
set(node_plugins "${node_plugins}my_components::AttachServer;$<TARGET_FILE:attachserver_component>\n")
set(node_plugins "${node_plugins}my_components::AttachClient;$<TARGET_FILE:attachclient_component>\n")

install(TARGETS
  preapproach_component
  attachserver_component
  attachclient_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

ament_package()
