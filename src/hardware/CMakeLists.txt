cmake_minimum_required(VERSION "3.9.0")

project("boarai_hardware" LANGUAGES CXX)

set(CMAKE_CXX_STANDARD "17")
set(CMAKE_CXX_STANDARD_REQUIRED YES)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)
set(CMAKE_EXPORT_COMPILE_COMMANDS YES)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../conan/cmake/Modules")

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE "RelWithDebInfo")
endif()

find_package("ament_cmake" REQUIRED)
find_package("Threads" REQUIRED)

if(EXISTS "${PROJECT_SOURCE_DIR}/conanfile.txt")
  include("conan")
  conan_check(REQUIRED)
  conan_add_remote(NAME bincrafters URL "https://api.bintray.com/conan/bincrafters/public-conan")
  conan_add_remote(NAME joakimono URL "https://api.bintray.com/conan/joakimono/conan")
  conan_add_remote(NAME fmorgner URL "https://api.bintray.com/conan/fmorgner/conan-public")
  conan_cmake_run(CONANFILE "conanfile.txt"
    BASIC_SETUP
    CMAKE_TARGETS
    BUILD "missing"
    NO_OUTPUT_DIRS
  )
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    "-Wall"
    "-Wextra"
    "-Wpedantic"
  )
endif()

### Prepare testing infrastructure

enable_testing()
set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
find_package("ament_lint_auto" REQUIRED)
ament_lint_auto_find_test_dependencies()

# ROS2 Dependencies

find_package("rclcpp" REQUIRED)
find_package("rclcpp_components" REQUIRED)
find_package("std_msgs" REQUIRED)

### libmodbus C++ interface

add_library("modbuscpp" SHARED
  "src/modbuscpp/client.cpp"
  "src/modbuscpp/connection.cpp"
  "src/modbuscpp/context.cpp"
  "src/modbuscpp/error.cpp"
  "src/modbuscpp/rtu_context.cpp"
  "src/modbuscpp/tcp_context.cpp"
)

target_include_directories("modbuscpp" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries("modbuscpp" "CONAN_PKG::libmodbus")

add_executable("modbuscpp_unittests"
  "test/src/modbuscpp/test_server.cpp"
  "test/src/modbuscpp/unittests.cpp"
)

target_include_directories("modbuscpp_unittests" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
)

target_link_libraries("modbuscpp_unittests"
  "modbuscpp"
  "CONAN_PKG::CUTE"
  "Threads::Threads"
)

add_test(NAME "modbuscpp_unittests"
  COMMAND "modbuscpp_unittests"
)

install(DIRECTORY "include/modbuscpp"
  DESTINATION "include"
)

install(TARGETS "modbuscpp"
  EXPORT "modbuscpp_export"
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "lib"
  INCLUDES DESTINATION "include"
)

### Roboteq Driver Library

add_library("roboteq"
  "src/roboteq/driver.cpp"
)

target_include_directories("roboteq" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries("roboteq"
  "modbuscpp"
)

install(DIRECTORY "include/roboteq"
  DESTINATION "include"
)

install(TARGETS "roboteq"
  EXPORT "roboteq_export"
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "lib"
  INCLUDES DESTINATION "include"
)

### Motor Control Node Configuration

add_library("motor_control" SHARED
  "src/motor_control.cpp"
)

ament_target_dependencies("motor_control"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)

target_link_libraries("motor_control"
  "modbuscpp"
)

install(TARGETS "motor_control"
  DESTINATION "lib"
)

rclcpp_components_register_nodes("motor_control" "boarai::hardware::motor_control")

### Run Unit-Tests

add_custom_target(run_all_tests ALL
  COMMAND ${CMAKE_CTEST_COMMAND} "--output-on-failure"
  DEPENDS "modbuscpp_unittests"
)

### Export Ament Package

ament_export_interfaces("modbuscpp_export" HAS_LIBRARY_TARGET)
ament_export_interfaces("roboteq_export" HAS_LIBRARY_TARGET)

ament_package()
