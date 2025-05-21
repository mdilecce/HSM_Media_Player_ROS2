cmake_minimum_required(VERSION 3.8)
project(media_system)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(pluginlib  REQUIRED)

pluginlib_export_plugin_description_file(media_state plugins.xml)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Speaker.msg"
  "action/Media.action"
)
ament_export_dependencies(rosidl_default_runtime)

#Adding interface reference in the same package to add as linked library
rosidl_get_typesupport_target(media_interface_cpp
  ${PROJECT_NAME} rosidl_typesupport_cpp)

include_directories(include)

add_executable(media_control
  src/media_control.cpp)
target_link_libraries(media_control "${media_interface_cpp}")
ament_target_dependencies(media_control
  "rclcpp"
  "rclcpp_action")

set(node_plugins "")

add_library(media_player_system SHARED
  src/media_player_system.cpp
  src/media_player_state.cpp
  src/media_power_state.cpp
  src/media_state.cpp)
target_link_libraries(media_player_system  "${media_interface_cpp}")
ament_target_dependencies(media_player_system
  "rclcpp"
  "rclcpp_action"
  "rclcpp_components"
  "pluginlib")

set(media_player_nodes
  "MediaState"
  "PowerOn"
  "PowerOff"
  "Play"
  "Pause"
  "Stop"
)

foreach(node ${media_player_nodes})
  rclcpp_components_register_nodes(media_player_system "media_system::${node}")
  set(node_plugins "${node_plugins}Media_system::${node};$<TARGET_FILE:media_player_system>\n")
endforeach()


add_library(media_listener SHARED
  src/media_listener.cpp)

target_link_libraries(media_listener  "${media_interface_cpp}")
ament_target_dependencies(media_listener
  "rclcpp"
  "rclcpp_components")
rclcpp_components_register_nodes(media_listener "media_system::MediaListener")
set(node_plugins "${node_plugins}Media_system::MediaListener;$<TARGET_FILE:media_listener>\n")
  
add_executable(media_system_main
  src/media_system.cpp)
target_link_libraries(media_system_main
  media_player_system
  media_listener)
ament_target_dependencies(media_system_main
  "rclcpp")

install(TARGETS
  media_player_system
  media_listener
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)

install(TARGETS
  media_control
  media_system_main
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)

install(
  DIRECTORY include/
  DESTINATION include
)

ament_export_include_directories(include)

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

ament_package()
