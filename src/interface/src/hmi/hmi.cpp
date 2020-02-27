#include "hmi/hmi.hpp"

#include "layer_constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace boarai::interface
{

  hmi::hmi(rclcpp::NodeOptions const & options)
      : fmt_node{HMI_NODE_NAME, LAYER_NAMESPACE, options}
  {
    log_info("HMI interface starting up");
  }

}  // namespace boarai::interface

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::interface::hmi)