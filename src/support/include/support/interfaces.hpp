#ifndef BOARAI_SUPPORT_INTERFACES_HPP
#define BOARAI_SUPPORT_INTERFACES_HPP

#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "support/messages.hpp"
#include "support/services.hpp"

namespace boarai
{

  auto constexpr ros_limit_namespace{"limit"};

  auto const default_limit_policy{rclcpp::QoS{1}.transient_local().reliable()};
  namespace control
  {
    auto constexpr ros_namespace{"/boarai/control"};
  }  // namespace control

  namespace estimation
  {
    auto constexpr ros_namespace{"/boarai/estimation"};

    namespace topic
    {
      using estimated_velocity_t = messages::PolarVelocity;
      auto constexpr estimated_velocity{"estimated_velocity"};
    }  // namespace topic
  }    // namespace estimation

  namespace hardware
  {
    auto constexpr ros_namespace{"/boarai/hardware"};

    namespace service
    {
      using get_maximum_angular_velocity_t = services::GetMaximumAngularVelocity;
      auto constexpr get_maximum_angular_velocity{"get_maximum_angular_velocity"};

      using set_drive_velocity_t = services::SetDriveVelocity;
      auto constexpr set_drive_velocity{"set_drive_velocity"};
    }  // namespace service

    namespace topic
    {
      using battery_voltage_t = messages::Voltage;
      auto constexpr battery_voltage{"battery_voltage"};

      using drive_velocity_t = messages::PolarVelocity;
      auto constexpr drive_velocity{"drive_velocity"};

      using global_position_t = sensor_msgs::msg::NavSatFix;
      auto constexpr global_position{"global_position"};
    }  // namespace topic

    namespace limit
    {
      using angular_velocity_t = messages::AngularVelocity;
      auto constexpr angular_velocity{"angular_velocity"};

      using linear_velocity_t = messages::LinearVelocity;
      auto constexpr linear_velocity{"linear_velocity"};
    }  // namespace limit
  }    // namespace hardware

  namespace intelligence
  {
    auto constexpr ros_namespace = "/boarai/intelligence";
  }  // namespace intelligence

  namespace interface
  {
    auto constexpr ros_namespace = "/boarai/interface";
  }  // namespace interface
}  // namespace boarai

#endif