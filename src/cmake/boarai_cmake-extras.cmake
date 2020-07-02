find_package("ament_cmake" REQUIRED)
find_package("boarai_support" REQUIRED)
find_package("rclcpp_components" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("std_msgs" REQUIRED)

include("${boarai_cmake_DIR}/add_composable_node.cmake")
include("${boarai_cmake_DIR}/ConanDependencies.cmake")