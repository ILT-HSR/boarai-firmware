#include "position_estimator/position_estimator.hpp"

#include "layer_constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

namespace boarai::estimation
{

  position_estimator::position_estimator(rclcpp::NodeOptions const & options)
      : fmt_node{POSITION_ESTIMATOR_NODE_NAME, LAYER_NAMESPACE, options}
  {
    log_info("position_estimator starting up");
  }

}  // namespace boarai::estimation

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::estimation::position_estimator)