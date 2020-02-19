#ifndef BOARAI_HARDWARE_GPS_PROVIDER_HPP
#define BOARAI_HARDWARE_GPS_PROVIDER_HPP

#include "rclcpp/rclcpp.hpp"
#include "support/fmt_node.hpp"

namespace boarai::hardware
{

  auto constexpr GPS_PROVIDER_NODE_NAME{"gps_provider"};

  struct gps_provider : fmt_node
  {
    using super = rclcpp::Node;

    explicit gps_provider(rclcpp::NodeOptions const & options);
  };

}  // namespace boarai::hardware

#endif