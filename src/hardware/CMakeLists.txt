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

### libmodbus C++ interface

add_library("modbuscpp" SHARED
  "src/modbuscpp/client.cpp"
  "src/modbuscpp/connection.cpp"
  "src/modbuscpp/context.cpp"
  "src/modbuscpp/error.cpp"
  "src/modbuscpp/rtu_context.cpp"
  "src/modbuscpp/tcp_context.cpp"
)

set(MODBUS_CPP_HEADERS
  "include/modbuscpp/address.hpp"
  "include/modbuscpp/client.hpp"
  "include/modbuscpp/connection.hpp"
  "include/modbuscpp/context.hpp"
  "include/modbuscpp/error.hpp"
  "include/modbuscpp/modbuscpp.hpp"
  "include/modbuscpp/rtu_context.hpp"
  "include/modbuscpp/tcp_context.hpp"
)

set_target_properties("modbuscpp" PROPERTIES PUBLIC_HEADER "${MODBUS_CPP_HEADERS}")

target_include_directories("modbuscpp" PUBLIC SYSTEM
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include/${PROJECT_NAME}>
)

target_link_libraries("modbuscpp"
  "CONAN_PKG::libmodbus"
)

add_executable("modbuscpp_unittests"
  "test/src/modbuscpp/unittests.cpp"
)

target_link_libraries("modbuscpp_unittests"
  "modbuscpp"
  "CONAN_PKG::CUTE"
)

add_test(NAME "modbuscpp_unittests"
  COMMAND "modbuscpp_unittests"
)

install(TARGETS "modbuscpp"
  DESTINATION "lib/${PROJECT_NAME}"
  PUBLIC_HEADER DESTINATION "include/modbuscpp"
)

### Main Node Configuration

find_package("rclcpp" REQUIRED)
find_package("std_msgs" REQUIRED)

add_executable("${PROJECT_NAME}"
  "src/hardware.cpp"
)

ament_target_dependencies("${PROJECT_NAME}"
  "rclcpp"
  "std_msgs"
)

target_link_libraries("${PROJECT_NAME}"
  "modbuscpp"
)

install(TARGETS "${PROJECT_NAME}"
  DESTINATION "lib/${PROJECT_NAME}"

)

### Run Unit-Tests

add_custom_target(run_all_tests ALL
  COMMAND ${CMAKE_CTEST_COMMAND} "--output-on-failure"
  DEPENDS "modbuscpp_unittests"
)

### Export Ament Package

ament_package()
