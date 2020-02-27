#ifndef BOARAI_ESTIMATION_POSITION_ESTIMATOR_HPP
#define BOARAI_ESTIMATION_POSITION_ESTIMATOR_HPP

#include "rclcpp/rclcpp.hpp"
#include "support/fmt_node.hpp"

namespace boarai::estimation
{

  struct position_estimator : fmt_node
  {
    using super = rclcpp::Node;

    explicit position_estimator(rclcpp::NodeOptions const & options);
  };

}  // namespace boarai::estimation

#endif