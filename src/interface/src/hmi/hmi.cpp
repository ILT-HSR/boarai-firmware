#include "hmi/hmi.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

namespace boarai::interface
{

  auto constexpr node_name{"hmi"};

  hmi::hmi(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("HMI interface starting up");
  }

}  // namespace boarai::interface

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::interface::hmi)