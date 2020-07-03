#ifndef BOARAI_SUPPORT_INTERFACES_HPP
#define BOARAI_SUPPORT_INTERFACES_HPP

#include "rclcpp/qos.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "std_msgs/msg/bool.hpp"
#include "support/messages.hpp"
#include "support/services.hpp"

namespace boarai
{
  auto const default_topic_policy{rclcpp::QoS{10}};

  auto constexpr ros_limit_namespace{"limit"};
  auto const default_limit_policy{rclcpp::QoS{1}.transient_local().reliable()};

  auto constexpr ros_status_namespace{"status"};
  auto const default_status_policy{rclcpp::QoS{1}.transient_local().reliable()};

  namespace control
  {
    auto constexpr ros_namespace{"/boarai/control"};

    namespace service
    {
      using set_target_velocity_t = services::SetVelocity;
      auto constexpr set_target_velocity{"set_target_velocity"};
    }  // namespace service
  }    // namespace control

  namespace estimation
  {
    auto constexpr ros_namespace{"/boarai/estimation"};

    namespace topic
    {
      using estimated_velocity_t = messages::PolarVelocity;
      auto constexpr estimated_velocity{"estimated_velocity"};

      using estimated_position_t = messages::Pose;
      auto constexpr estimated_position{"estimated_position"};
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

      using joystick_data_t = sensor_msgs::msg::Joy;
      auto constexpr joystick_data{"joystick_data"};

      using imu_orientation_t = messages::EulerOrientation;
      auto constexpr imu_orientation{"imu_orientation"};
    }  // namespace topic

    namespace limit
    {
      using angular_velocity_t = messages::AngularVelocity;
      auto constexpr angular_velocity{"angular_velocity"};

      using linear_velocity_t = messages::LinearVelocity;
      auto constexpr linear_velocity{"linear_velocity"};
    }  // namespace limit

    namespace status
    {
      using joystick_connected_t = std_msgs::msg::Bool;
      auto constexpr joystick_connected{"joystick_connected"};
    }  // namespace status
  }    // namespace hardware

  namespace intelligence
  {
    auto constexpr ros_namespace = "/boarai/intelligence";

    namespace service
    {
      using set_mode_t = services::SetMode;
      auto constexpr set_mode{"set_mode"};
    }  // namespace service
  }    // namespace intelligence

  namespace interface
  {
    auto constexpr ros_namespace = "/boarai/interface";

    namespace topic
    {
      using gamepad_input_t = messages::GamepadControls;
      auto constexpr gamepad_input = "gamepad";
    }  // namespace topic
  }    // namespace interface
}  // namespace boarai

#endif