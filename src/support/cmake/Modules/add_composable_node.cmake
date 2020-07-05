function(add_composable_node NAME)
  set(OPTIONS)
  set(SIGNLE_VALUE_ARGUMENTS)
  set(MULTI_VALUE_ARGUMENTS ROS_DEPENDENCIES CONAN_DEPENDENCIES)
  cmake_parse_arguments(COMPOSABLE_NODE "" "" "${MULTI_VALUE_ARGUMENTS}" ${ARGN})

  file(GLOB SOURCES RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS "src/${NAME}/*.cpp")
  file(GLOB HEADERS RELATIVE "${CMAKE_CURRENT_SOURCE_DIR}" CONFIGURE_DEPENDS "include/${NAME}/*.hpp" "include/${NAME}/*.h")

  add_library("${NAME}" SHARED
    ${SOURCES}
    ${HEADERS}
  )

  target_include_directories("${NAME}" PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
  )

  target_compile_features("${NAME}" PUBLIC
    "cxx_std_17"
  )

  target_compile_options("${NAME}" PRIVATE
    "-Wall"
    "-Wextra"
    "-Werror"
    "-pedantic-errors"
  )

  set_target_properties("${NAME}" PROPERTIES
    CXX_STANDARD_REQUIRED ON
    CXX_EXTENSIONS OFF
  )

  if(BOARAI_ENABLE_IPO)
    set_target_properties("${NAME}" PROPERTIES
      INTERPROCEDURAL_OPTIMIZATION ON
    )
  endif()

  ament_target_dependencies("${NAME}"
    "boarai_support"
    "rclcpp"
    "rclcpp_components"
    ${COMPOSABLE_NODE_ROS_DEPENDENCIES}
  )

  if(COMPOSABLE_NODE_CONAN_DEPENDENCIES)
    target_link_libraries("${NAME}"
      ${COMPOSABLE_NODE_CONAN_DEPENDENCIES}
    )
  endif()
endfunction()