cmake_minimum_required(VERSION "3.9.0")

project("boarai_control" LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake/Modules")

include("ConanDependencies")
include("CompilerWarnings")

#=== ROS2 Dependencies ===#

find_package("ament_cmake" REQUIRED)
find_package("boarai_support" REQUIRED)
find_package("boarai_estimation" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("rclcpp_components" REQUIRED)
find_package("std_msgs" REQUIRED)


#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
endif()

#=== velocity_control Node ===#

add_library("velocity_control" SHARED
  "src/velocity_control/velocity_control.cpp"
  "src/velocity_control/velocity_control_subscriptions.cpp"
)

target_include_directories("velocity_control" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies("velocity_control"
  "boarai_support"
  "boarai_estimation"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)

target_compile_features("velocity_control" PUBLIC
  "cxx_std_17"
)

set_target_properties("velocity_control" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
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

install(DIRECTORY
  "include/control"
  DESTINATION "include"
)

#=== Ament Packaging ===#

rclcpp_components_register_nodes("velocity_control" "boarai::control::velocity_control")

ament_export_interfaces("boarai_control" HAS_LIBRARY_TARGET)
ament_export_dependencies("boarai_support")
ament_package()

