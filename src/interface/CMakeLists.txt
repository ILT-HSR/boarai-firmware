cmake_minimum_required(VERSION "3.14.0")

project("boarai_interface" LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake/Modules")

include("ConanDependencies")
include("CompilerWarnings")

#=== ROS2 Dependencies ===#

find_package("ament_cmake" REQUIRED)
find_package("boarai_support" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("rclcpp_components" REQUIRED)
find_package("mavlink" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
endif()

#=== gamepad Node ===#

add_library("gamepad" SHARED
  "src/gamepad/gamepad.cpp"
)

target_include_directories("gamepad" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies("gamepad"
  "boarai_support"
  "rclcpp"
  "rclcpp_components"
)

target_compile_features("gamepad" PUBLIC
  "cxx_std_17"
)

set_target_properties("gamepad" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
)

#=== hmi Node ===#

add_library("hmi" SHARED
  "src/hmi/hmi.cpp"
)

target_include_directories("hmi" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies("hmi"
  "boarai_support"
  "mavlink"
  "rclcpp"
  "rclcpp_components"
)

target_compile_options("hmi" PUBLIC
  "-Wno-pedantic"
)

target_compile_features("hmi" PUBLIC
  "cxx_std_17"
)

set_target_properties("hmi" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
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
