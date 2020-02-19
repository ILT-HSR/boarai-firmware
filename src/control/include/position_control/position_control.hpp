#ifndef BOARAI_CONTROL_POSITION_CONTROL_HPP
#define BOARAI_CONTROL_POSITION_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "support/fmt_node.hpp"

namespace boarai::control
{

  auto constexpr POSITION_CONTROL_NODE_NAME{"hmi"};

  struct position_control : fmt_node
  {
    using super = rclcpp::Node;

    explicit position_control(rclcpp::NodeOptions const & options);
  };

}  // namespace boarai::control

#endif