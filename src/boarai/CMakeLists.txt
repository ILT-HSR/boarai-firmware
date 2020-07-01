cmake_minimum_required(VERSION "3.14.0")

project("boarai" LANGUAGES CXX)

find_package("ament_cmake" REQUIRED)

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

### Prepare testing infrastructure

enable_testing()
find_package("ament_lint_auto" REQUIRED)
ament_lint_auto_find_test_dependencies()

### Install Launch Files

install(DIRECTORY
  "launch"
  DESTINATION "share/${PROJECT_NAME}"
)

### Export Ament Package

ament_package()
