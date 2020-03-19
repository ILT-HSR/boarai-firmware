#include "gamepad/gamepad.hpp"

#include "rclcpp/node_options.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"
#include "support/string_utility.hpp"

#include <cmath>
#include <functional>
#include <limits>

using namespace std::placeholders;

namespace boarai::interface
{

  auto constexpr node_name{"gamepad"};

  gamepad::gamepad(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("gamepad interface starting up");
    m_set_mode_client = create_client<intelligence::service::set_mode_t>(
        join("/", intelligence::ros_namespace, intelligence::service::set_mode));

    start_publishers();
    start_subscriptions();
  }

  auto gamepad::start_subscriptions() -> void
  {
    m_joystick_connected_subscription = create_subscription<hardware::status::joystick_connected_t>(
        join("/", hardware::ros_namespace, ros_status_namespace, hardware::status::joystick_connected),
        default_status_policy,
        std::bind(&gamepad::on_joystick_connected_update, this, _1));
  }

  auto gamepad::start_publishers() -> void
  {
    m_gamepad_controls_publisher = create_publisher<topic::gamepad_input_t>(topic::gamepad_input, default_topic_policy);
  }

  auto gamepad::on_joystick_data_update(hardware::topic::joystick_data_t::SharedPtr new_data) -> void
  {
    auto static last_steering_input = std::numeric_limits<float>::infinity();
    auto static last_throttle_input = std::numeric_limits<float>::infinity();
    auto static const input_delta = 0.001f;

    auto joystick_positions = new_data->axes;

    if (std::abs(joystick_positions[1] - last_throttle_input) >= input_delta ||
        std::abs(joystick_positions[3] - last_steering_input) >= input_delta)
    {
      auto gamepad_input = topic::gamepad_input_t{}.set__throttle(joystick_positions[1]).set__steering(joystick_positions[3]);
      m_gamepad_controls_publisher->publish(gamepad_input);
      last_throttle_input = joystick_positions[1];
      last_steering_input = joystick_positions[3];
    }
  }

  auto gamepad::on_joystick_connected_update(hardware::status::joystick_connected_t::SharedPtr new_connection_state) -> void
  {
    auto request = std::make_shared<intelligence::service::set_mode_t::Request>();
    if (new_connection_state->data)
    {
      m_joystick_data_subscription = create_subscription<hardware::topic::joystick_data_t>(
          join("/", hardware::ros_namespace, hardware::topic::joystick_data),
          default_topic_policy,
          std::bind(&gamepad::on_joystick_data_update, this, _1));
      request->mode = "manual";
    }
    else
    {
      request->mode = "none";
      m_joystick_data_subscription.reset();
    }

    if (m_set_mode_client->service_is_ready())
    {
      log_debug("calling '{}' with mode '{}'", m_set_mode_client->get_service_name(), request->mode);
      m_set_mode_client->async_send_request(request);
    }
  }

}  // namespace boarai::interface

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::interface::gamepad)