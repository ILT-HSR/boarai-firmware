#include "support/fmt_node.hpp"

namespace boarai
{

  fmt_node::fmt_node(const std::string & node_name, const rclcpp::NodeOptions & options)
      : Node{node_name, options}
  {
  }

  fmt_node::fmt_node(const std::string & node_name, const std::string & namespace_, const rclcpp::NodeOptions & options)
      : Node{node_name, namespace_, options}
  {
  }

}  // namespace boarai