#include "cortex/cortex.hpp"

#include "layer_constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace boarai::intelligence
{

  cortex::cortex(rclcpp::NodeOptions const & options)
      : fmt_node{CORTEX_NODE_NAME, LAYER_NAMESPACE, options}
  {
    log_info("cortex starting up");
  }

}  // namespace boarai::intelligence

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::intelligence::cortex)