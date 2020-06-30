
#include "imu/imu.hpp"

#include "imu/bno055.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

#include <memory>

auto constexpr node_name{"imu"};

namespace boarai::hardware
{

  imu::imu(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
      , m_device{std::make_unique<bno055>("/dev/i2c-0")}
  {
    auto orientation = m_device->euler_orientation();
    log_info("Orientation == [ {}, {}, {} ]", orientation.heading, orientation.pitch, orientation.roll);
  }

}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::imu)