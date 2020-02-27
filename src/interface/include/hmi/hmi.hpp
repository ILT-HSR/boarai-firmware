#ifndef BOARAI_INTERFACE_HMI_HPP
#define BOARAI_INTERFACE_HMI_HPP

#include "rclcpp/rclcpp.hpp"
#include "support/fmt_node.hpp"

namespace boarai::interface
{

  auto constexpr HMI_NODE_NAME{"hmi"};

  struct hmi : fmt_node
  {
    using super = rclcpp::Node;

    explicit hmi(rclcpp::NodeOptions const & options);
  };

}  // namespace boarai::interface

#endif