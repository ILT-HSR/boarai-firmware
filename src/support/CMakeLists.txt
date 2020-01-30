cmake_minimum_required(VERSION "3.9.0")

project("boarai_support" LANGUAGES CXX)

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

find_package("rclcpp" REQUIRED)

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

### Prepare testing infrastructure

enable_testing()
set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")
find_package("ament_lint_auto" REQUIRED)
ament_lint_auto_find_test_dependencies()

### Support Library

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

install(DIRECTORY "include/support"
  DESTINATION "include"
)

install(TARGETS "support"
  EXPORT "support"
  ARCHIVE DESTINATION "lib"
  LIBRARY DESTINATION "lib"
  INCLUDES DESTINATION "include"
)

install(DIRECTORY
  "${PROJECT_BINARY_DIR}/conan_imported_libs/"
  DESTINATION "lib"
)

### Export Ament Package

ament_export_include_directories("include" ${CONAN_INCLUDE_DIRS})
ament_export_interfaces("support")
ament_export_libraries("support")

ament_package()
