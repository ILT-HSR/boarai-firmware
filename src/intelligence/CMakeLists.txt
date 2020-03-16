cmake_minimum_required(VERSION "3.9.0")

project("boarai_intelligence" LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake/Modules")

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

add_library("mode_controller" SHARED
  "src/mode_controller/mode_controller_subscriptions.cpp"
  "src/mode_controller/mode_controller_timers.cpp"
  "src/mode_controller/mode_controller.cpp"
)

target_include_directories("mode_controller" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies("mode_controller"
  "boarai_support"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)

target_compile_features("mode_controller" PUBLIC
  "cxx_std_17"
)

set_target_properties("mode_controller" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
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
