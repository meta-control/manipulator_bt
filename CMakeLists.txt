cmake_minimum_required(VERSION 3.5)
project(manipulator_bt)

set(CMAKE_BUILD_TYPE DEBUG)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(ros2_knowledge_graph REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(behaviortree_cpp_v3 REQUIRED)
find_package(mros2_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(system_modes REQUIRED)

include_directories(
  include
)

set(dependencies
  rclcpp
  rclcpp_action
  rclcpp_lifecycle
  ros2_knowledge_graph
  ament_index_cpp
  nav2_msgs
  behaviortree_cpp_v3
  mros2_msgs
  std_srvs
  system_modes
)


add_library(manipulator_bt_navigate_to_wp_bt_node SHARED
  src/behavior_tree_nodes/NavigateToWp.cpp
)
list(APPEND plugin_libs manipulator_bt_navigate_to_wp_bt_node)

add_library(manipulator_bt_reconfigure_bt_node SHARED
  src/behavior_tree_nodes/Reconfigure.cpp
)
list(APPEND plugin_libs manipulator_bt_reconfigure_bt_node)

add_library(manipulator_bt_check_component_bt_node SHARED
  src/behavior_tree_nodes/CheckComponent.cpp
)
list(APPEND plugin_libs manipulator_bt_check_component_bt_node)

add_library(manipulator_bt_pick_bt_node SHARED
  src/behavior_tree_nodes/Pick.cpp
)
list(APPEND plugin_libs manipulator_bt_pick_bt_node)

add_library(manipulator_bt_place_bt_node SHARED
  src/behavior_tree_nodes/Place.cpp
)
list(APPEND plugin_libs manipulator_bt_place_bt_node)


foreach(bt_plugin ${plugin_libs})
  ament_target_dependencies(${bt_plugin} ${dependencies})
  target_compile_definitions(${bt_plugin} PRIVATE BT_PLUGIN_EXPORT)
endforeach()

add_executable(bt_controller src/bt_controller.cpp)
ament_target_dependencies(bt_controller ${dependencies})
target_link_libraries(bt_controller)

install(DIRECTORY behavior_trees DESTINATION share/${PROJECT_NAME})
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

install(TARGETS
  ${plugin_libs}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(TARGETS
  bt_controller
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
