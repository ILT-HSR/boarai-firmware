#ifndef BOARAI_VELOCITY_POSITION_CONTROL_HPP
#define BOARAI_VELOCITY_POSITION_CONTROL_HPP

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "support/fmt_node.hpp"

namespace boarai::control
{
  struct velocity_control : fmt_node
  {
    using super = rclcpp::Node;

    explicit velocity_control(rclcpp::NodeOptions const & options);
  };

}  // namespace boarai::control

#endif