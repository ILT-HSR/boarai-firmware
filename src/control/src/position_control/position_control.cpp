#include "position_control/position_control.hpp"

#include "layer_constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace boarai::control
{

  position_control::position_control(rclcpp::NodeOptions const & options)
      : fmt_node{POSITION_CONTROL_NODE_NAME, LAYER_NAMESPACE, options}
  {
    log_info("position_control starting up");
  }

}  // namespace boarai::control

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::control::position_control)