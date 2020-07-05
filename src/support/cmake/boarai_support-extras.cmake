find_package("ament_cmake" REQUIRED)
find_package("rclcpp_components" REQUIRED)
find_package("rclcpp" REQUIRED)
find_package("std_msgs" REQUIRED)
find_package("sensor_msgs" REQUIRED)

include("${boarai_support_DIR}/add_composable_node.cmake")
include("${boarai_support_DIR}/ConanDependencies.cmake")