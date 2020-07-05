cmake_minimum_required(VERSION "3.14.0")

project("boarai_estimation" LANGUAGES CXX)

#=== ROS2 Dependencies ===#

find_package("boarai_support" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
endif()

#=== position_estimator Node ===#

add_composable_node("position_estimator")

#=== velocity_estimator Node ===#

add_composable_node("velocity_estimator")

#=== Package Installation ===#

install(TARGETS
  # nodes
  "position_estimator"
  "velocity_estimator"

  EXPORT "boarai_estimation"
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

rclcpp_components_register_nodes("position_estimator" "boarai::estimation::position_estimator")
rclcpp_components_register_nodes("velocity_estimator" "boarai::estimation::velocity_estimator")

ament_export_interfaces("boarai_estimation" HAS_LIBRARY_TARGET)
ament_export_dependencies("boarai_support")
ament_package()