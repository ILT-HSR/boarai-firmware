cmake_minimum_required(VERSION "3.14.0")

project("boarai_control" LANGUAGES CXX)

#=== ROS2 Dependencies ===#

find_package("boarai_support" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
endif()

#=== velocity_control Node ===#

add_composable_node("velocity_control"
  ROS_DEPENDENCIES "std_msgs"
)

#=== Package Installation ===#

install(TARGETS
  # nodes
  "velocity_control"

  EXPORT "boarai_control"
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "lib"
  INCLUDES DESTINATION "include"
)

install(DIRECTORY
  "launch"
  DESTINATION "share/${PROJECT_NAME}"
)

#=== Ament Packaging ===#

rclcpp_components_register_nodes("velocity_control" "boarai::control::velocity_control")

ament_export_interfaces("boarai_control" HAS_LIBRARY_TARGET)
ament_export_dependencies("boarai_support")
ament_package()

