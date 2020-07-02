file(GLOB CONANFILE RELATIVE "${PROJECT_SOURCE_DIR}" CONFIGURE_DEPENDS "conanfile.txt")

if(CONANFILE)
  if(EXISTS "${boarai_cmake_DIR}/conan.cmake")
    include("${boarai_cmake_DIR}/conan.cmake")
  else()
    include("conan")
  endif()
  conan_check(REQUIRED)
  conan_add_remote(NAME bincrafters URL "https://api.bintray.com/conan/bincrafters/public-conan")
  conan_cmake_run(CONANFILE "conanfile.txt"
    BASIC_SETUP
    CMAKE_TARGETS
    BUILD "missing"
    NO_OUTPUT_DIRS
  )
endif()