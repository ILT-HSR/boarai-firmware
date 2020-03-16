cmake_minimum_required(VERSION "3.9.0")

project("boarai_support" LANGUAGES CXX C)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake/Modules")

include("ConanDependencies")
include("CompilerWarnings")

#=== ROS2 Dependencies ===#

find_package("ament_cmake" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("rosidl_default_generators" REQUIRED)
find_package("sensor_msgs" REQUIRED)
find_package("std_msgs" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")

  add_custom_target(run_all_tests ALL
    COMMAND ${CMAKE_CTEST_COMMAND} "--output-on-failure"
    DEPENDS "support_unittests"
  )
endif()

#=== boarAI Support Library ===#

add_library("support" SHARED
  "src/fmt_node.cpp"
)

target_include_directories("support" PUBLIC
  $<BUILD_INTERFACE:${PROJECT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_compile_features("support" PUBLIC
  "cxx_std_17"
)

target_link_libraries("support"
  "CONAN_PKG::fmt"
)

ament_target_dependencies("support"
  "rclcpp"
  "sensor_msgs"
  "std_msgs"
)

rosidl_generate_interfaces("${PROJECT_NAME}"
  "msg/AngularVelocity.msg"
  "msg/LinearVelocity.msg"
  "msg/Polar2D.msg"
  "msg/PolarVelocity.msg"
  "msg/Voltage.msg"
  "srv/GetMaximumAngularVelocity.srv"
  "srv/SetDriveVelocity.srv"
  "srv/SetVelocity.srv"
  DEPENDENCIES builtin_interfaces
)

#=== boarAI Support Library Tests ===#

add_executable("support_unittests"
  "test/src/support/unittests.cpp"
)

target_include_directories("support_unittests" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
)

target_link_libraries("support_unittests"
  "support"
  "CONAN_PKG::CUTE"
)

add_test(NAME "support_unittests"
  COMMAND "support_unittests"
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

ament_export_dependencies("rosidl_default_runtime" "std_msgs" "sensor_msgs")
ament_export_include_directories("include" ${CONAN_INCLUDE_DIRS_FMT})
ament_export_interfaces("boarai_support")
ament_export_libraries("support")

ament_package()