#include "gps_provider/gps_provider.hpp"

#include "layer_constants.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <libgpsmm.h>

namespace boarai::hardware
{

  gps_provider::gps_provider(rclcpp::NodeOptions const & options)
      : fmt_node{GPS_PROVIDER_NODE_NAME, LAYER_NAMESPACE, options}
  {
    log_info("gps_provider starting up");
  }

}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::gps_provider)