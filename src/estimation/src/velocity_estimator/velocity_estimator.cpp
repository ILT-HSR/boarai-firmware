#include "velocity_estimator/velocity_estimator.hpp"

#include "estimation/layer_interface.hpp"
#include "hardware/layer_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/string_utility.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::estimation
{

  auto constexpr node_name{"velocity_estimator"};

  velocity_estimator::velocity_estimator(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("position_estimator starting up");

    start_publishers();
    start_subscriptions();
  }

  auto velocity_estimator::start_publishers() -> void
  {
    m_estimated_velocity_publisher = create_publisher<topic::estimated_velocity_t>(topic::estimated_velocity, 10);
  }

  auto velocity_estimator::start_subscriptions() -> void
  {
    m_drive_velocity_subscription = create_subscription<hardware::topic::drive_velocity_t>(
        join("/", hardware::ros_namespace, hardware::topic::drive_velocity),
        10,
        std::bind(&velocity_estimator::on_drive_velocity_update, this, _1));
  }

  auto velocity_estimator::on_drive_velocity_update(hardware::topic::drive_velocity_t::SharedPtr new_velocity) -> void
  {
    auto [linear_velocity, angular_velocity] = new_velocity->value;

    m_linear_drive_velocity = (m_linear_drive_velocity * 5 + linear_velocity) / (5 + 1);
    m_angular_drive_velocity = (m_angular_drive_velocity * 5 + angular_velocity) / (5 + 1);

    auto msg = topic::estimated_velocity_t{};
    auto velocity = topic::estimated_velocity_t::_value_type{};
    velocity.r = m_linear_drive_velocity;
    velocity.phi = m_angular_drive_velocity;
    msg.value = velocity;
    m_estimated_velocity_publisher->publish(msg);
  }

}  // namespace boarai::estimation

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::estimation::velocity_estimator)