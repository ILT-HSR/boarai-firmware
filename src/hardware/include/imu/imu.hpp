#ifndef BOARAI_HARDWARE_IMU_HPP
#define BOARAI_HARDWARE_IMU_HPP

#include "imu/imu_device.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"
#include "support/to_string.hpp"

#include <cstdint>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace boarai::hardware
{

  struct imu : fmt_node
  {
    using super = rclcpp::Node;

    explicit imu(rclcpp::NodeOptions const & options);

  private:
    std::unique_ptr<imu_device> m_device{};
  };

}  // namespace boarai::hardware

#endif