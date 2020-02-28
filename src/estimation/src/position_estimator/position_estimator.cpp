#include "position_estimator/position_estimator.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

namespace boarai::estimation
{

  auto constexpr node_name{"position_estimator"};

  position_estimator::position_estimator(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("position_estimator starting up");
  }

}  // namespace boarai::estimation

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::estimation::position_estimator)