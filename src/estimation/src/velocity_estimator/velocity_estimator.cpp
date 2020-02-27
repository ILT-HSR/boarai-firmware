#include "velocity_estimator/velocity_estimator.hpp"

#include "estimation/layer_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace boarai::estimation
{

  auto constexpr node_name{"velocity_estimator"};

  velocity_estimator::velocity_estimator(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("position_estimator starting up");
  }

}  // namespace boarai::estimation

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::estimation::velocity_estimator)