#include "cortex/cortex.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

namespace boarai::intelligence
{

  auto constexpr node_name{"cortex"};

  cortex::cortex(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("cortex starting up");
  }

}  // namespace boarai::intelligence

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::intelligence::cortex)