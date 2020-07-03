
#include "imu/imu.hpp"

#include "imu/bno055.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

#include <exception>
#include <memory>

auto constexpr node_name{"imu"};

namespace boarai::hardware
{

  imu::imu(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
      , m_clock{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)}
  {
    try
    {
      m_device = std::make_unique<bno055>("/dev/i2c-0");
      start_publishers();
      start_timers();
    }
    catch (std::exception const & e)
    {
      log_error("Failed to initialize imu! reason: {0}", e.what());
    }
  }

}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::imu)
