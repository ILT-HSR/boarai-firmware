file(GLOB CONANFILE RELATIVE "${PROJECT_SOURCE_DIR}" CONFIGURE_DEPENDS "conanfile.txt")

if(CONANFILE)
  if(EXISTS "${boarai_support_DIR}/conan.cmake")
    include("${boarai_support_DIR}/conan.cmake")
  else()
    include("conan")
  endif()
  conan_check(REQUIRED)
  conan_add_remote(NAME bincrafters URL "https://api.bintray.com/conan/bincrafters/public-conan")
  conan_add_remote(NAME fmorgner URL "https://api.bintray.com/conan/fmorgner/conan-public")
  conan_add_remote(NAME joakimono URL "https://api.bintray.com/conan/joakimono/conan")
  conan_cmake_run(CONANFILE "conanfile.txt"
    BASIC_SETUP
    CMAKE_TARGETS
    BUILD "missing"
    NO_OUTPUT_DIRS
  )
endif()