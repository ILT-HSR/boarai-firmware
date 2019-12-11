cmake_minimum_required(VERSION "3.9.0")

project("boarai_intelligence" LANGUAGES CXX)

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

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

### Node/Library Configuration

find_package("rclcpp" REQUIRED)
find_package("std_msgs" REQUIRED)

add_executable("${PROJECT_NAME}"
  "src/intelligence.cpp"
)

ament_target_dependencies("${PROJECT_NAME}"
  "rclcpp"
  "std_msgs"
)

install(TARGETS "${PROJECT_NAME}"
  DESTINATION "lib/${PROJECT_NAME}"
)

### Export Ament Package

ament_package()