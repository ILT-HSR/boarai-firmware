#include "velocity_control/velocity_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

namespace boarai::control
{
  auto constexpr node_name{"velocity_control"};

  velocity_control::velocity_control(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("velocity_control starting up");
    start_subscriptions();
  }

}  // namespace boarai::control

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::control::velocity_control)