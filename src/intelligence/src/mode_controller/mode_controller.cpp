#include "mode_controller/mode_controller.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

namespace boarai::intelligence
{
  auto constexpr node_name{"mode_controller"};

  mode_controller::mode_controller(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("mode_controller starting up");
    start_timers();
  }
}  // namespace boarai::intelligence

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::intelligence::mode_controller)