cmake_minimum_required(VERSION "3.9.0")

project("boarai_hardware" LANGUAGES CXX)

list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/../../cmake/Modules")

include("ConanDependencies")
include("CompilerWarnings")

#=== System Dependencies ===#

find_package("PkgConfig" REQUIRED)
find_package("Threads" REQUIRED)

pkg_check_modules("libgps" REQUIRED IMPORTED_TARGET "libgps")

#=== ROS2 Dependencies ===#

find_package("ament_cmake" REQUIRED)
find_package("boarai_support" REQUIRED)
find_package("rclcpp_components" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("sensor_msgs" REQUIRED)
find_package("std_msgs" REQUIRED)

#=== Testing Setup ===#

if(BUILD_TESTING)
  find_package("ament_lint_auto" REQUIRED)
  ament_lint_auto_find_test_dependencies()
  set(ament_cmake_clang_format_CONFIG_FILE "${PROJECT_SOURCE_DIR}/../../.clang-format")

  add_custom_target(run_all_tests ALL
    COMMAND ${CMAKE_CTEST_COMMAND} "--output-on-failure"
    DEPENDS "modbuscpp_unittests"
  )
endif()


#=== modbuscpp Library ===#

add_library("modbuscpp" SHARED
  "src/modbuscpp/client.cpp"
  "src/modbuscpp/connection.cpp"
  "src/modbuscpp/context.cpp"
  "src/modbuscpp/error.cpp"
  "src/modbuscpp/rtu_context.cpp"
  "src/modbuscpp/tcp_context.cpp"
)

target_include_directories("modbuscpp" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries("modbuscpp" "CONAN_PKG::libmodbus")

target_compile_features("modbuscpp" PUBLIC
  "cxx_std_17"
)

set_target_properties("modbuscpp" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
)

#=== modbuscpp Library Tests ===#

add_executable("modbuscpp_unittests"
  "test/src/modbuscpp/test_server.cpp"
  "test/src/modbuscpp/unittests.cpp"
)

target_include_directories("modbuscpp_unittests" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/test/include>
)

target_link_libraries("modbuscpp_unittests"
  "modbuscpp"
  "CONAN_PKG::CUTE"
  "Threads::Threads"
)

add_test(NAME "modbuscpp_unittests"
  COMMAND "modbuscpp_unittests"
)

#=== roboteq Library ===#

add_library("roboteq" SHARED
  "src/roboteq/driver.cpp"
  "src/roboteq/command.cpp"
)

target_include_directories("roboteq" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries("roboteq"
  "modbuscpp"
)

target_compile_features("roboteq" PUBLIC
  "cxx_std_17"
)

set_target_properties("roboteq" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
)

#=== tank_drive Node ===#

add_library("tank_drive" SHARED
  "src/tank_drive/tank_drive.cpp"
)

target_include_directories("tank_drive" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries("tank_drive"
  "modbuscpp"
  "roboteq"
)

ament_target_dependencies("tank_drive"
  "boarai_support"
  "rclcpp"
  "rclcpp_components"
  "std_msgs"
)

target_compile_features("tank_drive" PUBLIC
  "cxx_std_17"
)

set_target_properties("tank_drive" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
)

#=== gps_provider Node ===#

add_library("gps_provider" SHARED
  "src/gps_provider/gps_provider.cpp"
  "src/gps_provider/gpsd_client.cpp"
)

target_include_directories("gps_provider" PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries("gps_provider"
  "PkgConfig::libgps"
)

ament_target_dependencies("gps_provider"
  "boarai_support"
  "rclcpp"
  "rclcpp_components"
  "sensor_msgs"
  "std_msgs"
)

target_compile_features("gps_provider" PUBLIC
  "cxx_std_17"
)

set_target_properties("gps_provider" PROPERTIES
  CXX_STANDARD_REQUIRED ON
  CXX_EXTENSIONS OFF
  INTERPROCEDURAL_OPTIMIZATION ON
)

#=== Package Installation ===#

install(TARGETS
  # libraries
  "modbuscpp"
  "roboteq"

  # nodes
  "tank_drive"
  "gps_provider"

  EXPORT "boarai_hardware"
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "lib"
  INCLUDES DESTINATION "include"
)

install(DIRECTORY
  "launch"
  DESTINATION "share/${PROJECT_NAME}"
)

#=== Ament Packaging ===#

rclcpp_components_register_nodes("tank_drive" "boarai::hardware::tank_drive")
rclcpp_components_register_nodes("gps_provider" "boarai::hardware::gps_provider")

ament_export_interfaces("boarai_hardware" HAS_LIBRARY_TARGET)

ament_package()
