cmake_minimum_required(VERSION "3.14.0")

project("boarai_intelligence" LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake/Modules")

include("AddComposableNode")
include("ConanDependencies")
include("CompilerWarnings")

#=== ROS2 Dependencies ===#

find_package("ament_cmake" REQUIRED)
find_package("boarai_support" REQUIRED)
find_package("rclcpp_components" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("std_msgs" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
endif()

#=== mode_controller Node ===#

add_composable_node("mode_controller")

ament_target_dependencies("mode_controller"
  "std_msgs"
)

#=== Package Installation ===#

install(TARGETS
  # nodes
  "mode_controller"

  EXPORT "boarai_intelligence"
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

rclcpp_components_register_nodes("mode_controller" "boarai::intelligence::mode_controller")

ament_export_interfaces("boarai_intelligence" HAS_LIBRARY_TARGET)

ament_package()
