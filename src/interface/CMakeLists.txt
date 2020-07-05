cmake_minimum_required(VERSION "3.14.0")

project("boarai_interface" LANGUAGES CXX)

#=== ROS2 Dependencies ===#

find_package("boarai_support" REQUIRED)
find_package("mavlink" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
endif()

#=== gamepad Node ===#

add_composable_node("gamepad")

#=== hmi Node ===#

add_composable_node("hmi"
  ROS_DEPENDENCIES "mavlink"
)

target_compile_options("hmi" PUBLIC
  "-Wno-pedantic"
)

#=== Package Installation ===#

install(TARGETS
  # nodes
  "hmi"
  "gamepad"

  EXPORT "boarai_interface"
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

rclcpp_components_register_nodes("gamepad" "boarai::interface::gamepad")
rclcpp_components_register_nodes("hmi" "boarai::interface::hmi")

ament_export_interfaces("boarai_interface" HAS_LIBRARY_TARGET)
ament_export_dependencies("boarai_support")

ament_package()
