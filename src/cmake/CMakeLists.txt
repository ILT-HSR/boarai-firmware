cmake_minimum_required(VERSION "3.14.0")

project("boarai_cmake"
  LANGUAGES NONE
)

find_package("ament_cmake" REQUIRED)

ament_package(CONFIG_EXTRAS "boarai_cmake-extras.cmake")

install(DIRECTORY "cmake"
  DESTINATION "share/${PROJECT_NAME}"
)