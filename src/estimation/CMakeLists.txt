cmake_minimum_required(VERSION "3.9.0")

project("boarai_estimation" LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake/Modules")

include("ConanDependencies")
include("CompilerWarnings")

#=== ROS2 Dependencies ===#

find_package("ament_cmake" REQUIRED)
find_package("boarai_support" REQUIRED)
find_package("boarai_hardware" REQUIRED)
find_package("rclcpp_components" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("std_msgs" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
endif()

#=== position_estimator Node ===#

add_library("position_estimator" SHARED
  "src/position_estimator/position_estimator.cpp"
)

target_include_directories("position_estimator" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies("position_estimator"
  "boarai_support"
  "rclcpp"
  "rclcpp_components"
)

target_compile_features("position_estimator" PUBLIC
  "cxx_std_17"
)

set_target_properties("position_estimator" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
)

#=== velocity_estimator Node ===#

add_library("velocity_estimator" SHARED
  "src/velocity_estimator/velocity_estimator.cpp"
)

target_include_directories("velocity_estimator" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies("velocity_estimator"
  "boarai_hardware"
  "boarai_support"
  "rclcpp_components"
  "rclcpp"
)

target_compile_features("velocity_estimator" PUBLIC
  "cxx_std_17"
)

set_target_properties("velocity_estimator" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
)

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

install(DIRECTORY
  "include/estimation"
  DESTINATION "include"
)

#=== Ament Packaging ===#

rclcpp_components_register_nodes("position_estimator" "boarai::estimation::position_estimator")
rclcpp_components_register_nodes("velocity_estimator" "boarai::estimation::velocity_estimator")

ament_export_interfaces("boarai_estimation" HAS_LIBRARY_TARGET)
ament_export_include_directories("include" ${CONAN_INCLUDE_DIRS})
ament_export_dependencies("boarai_support")
ament_package()