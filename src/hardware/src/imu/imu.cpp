#include "imu/imu.hpp"

#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

auto constexpr node_name{"imu"};

namespace boarai::hardware
{

  imu::imu(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
  }

}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::imu)