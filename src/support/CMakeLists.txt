cmake_minimum_required(VERSION "3.9.0")

project("boarai_support" LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake/Modules")

include("ConanDependencies")
include("CompilerWarnings")

#=== ROS2 Dependencies ===#

find_package("ament_cmake" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("rosidl_default_generators" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
endif()

#=== boarAI Support Library ===#

add_library("support" SHARED
  "src/fmt_node.cpp"
)

target_include_directories("support" PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries("support"
  "CONAN_PKG::fmt"
)

ament_target_dependencies("support"
  "rclcpp"
)

rosidl_generate_interfaces("${PROJECT_NAME}"
  "msg/Polar2D.msg"
  "msg/PolarVelocity.msg"
  "srv/SetDriveVelocity.srv"
  DEPENDENCIES builtin_interfaces
)

#=== Package Installation ===#

install(DIRECTORY "include/support"
  DESTINATION "include"
)

install(TARGETS "support"
  EXPORT "boarai_support"
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  INCLUDES DESTINATION "include"
)

install(DIRECTORY
  "${PROJECT_BINARY_DIR}/conan_imported_libs/"
  DESTINATION "lib"
)


#=== Ament Packaging ===#

ament_export_dependencies("rosidl_default_runtime")
ament_export_include_directories("include" ${CONAN_INCLUDE_DIRS})
ament_export_interfaces("boarai_support")
ament_export_libraries("support")

ament_package()