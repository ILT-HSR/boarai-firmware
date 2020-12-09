#include "tank_drive_debug/tank_drive_debug.hpp"

#include "rclcpp/node_options.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/fmt_node.hpp"
#include "support/lerp.hpp"
#include "support/string_utility.hpp"

#include <string>

namespace boarai::hardware
{
  namespace
  {
    auto constexpr node_name{"tank_drive"};

    auto const angular_velocity_limit_topic{join("/", ros_limit_namespace, join("_", limit::angular_velocity, node_name))};
    auto const linear_velocity_limit_topic{join("/", ros_limit_namespace, join("_", limit::linear_velocity, node_name))};
  }  // namespace

  tank_drive_debug::tank_drive_debug(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    declare_parameters();
    start_services();
    start_publishers();
    start_timers();
    publish_limits();
  }

  auto tank_drive_debug::maximum_angular_velocity() -> double
  {
    assert(wheel_spacing() > 0.0);
    auto turn_circumference = wheel_spacing() * M_PI;
    return 360.0 * maximum_linear_velocity() / turn_circumference;
  }

  auto tank_drive_debug::publish_limits() -> void
  {
    m_angular_velocity_limit_publisher =
        create_publisher<limit::angular_velocity_t>(angular_velocity_limit_topic, default_limit_policy);
    m_angular_velocity_limit_publisher->publish(messages::AngularVelocity{}.set__value(maximum_angular_velocity()));

    m_linear_velocity_limit_publisher =
        create_publisher<limit::linear_velocity_t>(linear_velocity_limit_topic, default_limit_policy);
    m_linear_velocity_limit_publisher->publish(messages::LinearVelocity{}.set__value(maximum_linear_velocity()));
  }

}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::tank_drive_debug)
