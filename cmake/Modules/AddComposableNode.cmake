function(add_composable_node NAME)
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
  )
endfunction()