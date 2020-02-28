#ifndef BOARAI_INTELLIGENCE_CORTEX_HPP
#define BOARAI_INTELLIGENCE_CORTEX_HPP

#include "rclcpp/rclcpp.hpp"
#include "support/fmt_node.hpp"

namespace boarai::intelligence
{

  struct cortex : fmt_node
  {
    using super = rclcpp::Node;

    explicit cortex(rclcpp::NodeOptions const & options);
  };

}  // namespace boarai::intelligence

#endif