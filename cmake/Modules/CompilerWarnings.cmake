if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(
    "-Wall"
    "-Wextra"
    "-Werror"
    "-pedantic-errors"
  )
endif()