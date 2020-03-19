#ifndef BOARAI_INTERFACE_GAMEPAD_HPP
#define BOARAI_INTERFACE_GAMEPAD_HPP

#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp/subscription.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"

namespace boarai::interface
{
  struct gamepad : fmt_node
  {
    using super = rclcpp::Node;

    explicit gamepad(rclcpp::NodeOptions const & options);

  private:
    auto start_subscriptions() -> void;
    auto start_publishers() -> void;

    auto on_joystick_data_update(hardware::topic::joystick_data_t::SharedPtr new_data) -> void;
    auto on_joystick_connected_update(hardware::status::joystick_connected_t::SharedPtr new_connection_state) -> void;

    rclcpp::SubscriptionBase::SharedPtr m_joystick_data_subscription{};
    rclcpp::SubscriptionBase::SharedPtr m_joystick_connected_subscription{};
    rclcpp::Publisher<topic::gamepad_input_t>::SharedPtr m_gamepad_controls_publisher{};
    rclcpp::Client<intelligence::service::set_mode_t>::SharedPtr m_set_mode_client{};
  };

}  // namespace boarai::interface

#endif