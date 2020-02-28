#ifndef BOARAI_CONTROL_VELOCITY_CONTROL_HPP
#define BOARAI_CONTROL_VELOCITY_CONTROL_HPP

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"

namespace boarai::control
{
  struct velocity_control : fmt_node
  {
    using super = rclcpp::Node;

    explicit velocity_control(rclcpp::NodeOptions const & options);

  private:
    auto start_subscriptions() -> void;

    auto on_estimated_velocity_update(estimation::topic::estimated_velocity_t::SharedPtr new_velocity) -> void;

    rclcpp::Subscription<estimation::topic::estimated_velocity_t>::SharedPtr m_estimated_velocity_subscription{};
  };

}  // namespace boarai::control

#endif