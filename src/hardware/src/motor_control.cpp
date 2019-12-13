#include "motor_control.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <modbuscpp/address.hpp>
#include <modbuscpp/context.hpp>

#include <chrono>
#include <cstdint>
#include <functional>
#include <string>

namespace boarai::hardware
{
  namespace
  {
    auto make_context(std::string address, std::uint16_t port) -> modbus::context
    {
      using namespace std::chrono_literals;

      auto tcp_context = modbus::tcp_context{address, port};
      tcp_context.slave_id(1);
      tcp_context.response_timeout(1s);

      return std::move(tcp_context);
    }
  }  // namespace

  motor_control::motor_control(rclcpp::NodeOptions const & options)
      : Node{"motor_control", "hardware", options}
      , m_driver_connection{make_context("192.168.1.20", 502)}
  {
    m_subscription =
        create_subscription<std_msgs::msg::Float32>("linear_velocity",
                                                    10,
                                                    std::bind(&motor_control::handle_message, this, std::placeholders::_1));
  }

  auto motor_control::handle_message(std_msgs::msg::Float32::SharedPtr message) -> void
  {
    using namespace modbus::modbus_literals;

    RCLCPP_INFO(get_logger(), "Received: '%f'", message->data);

    auto velocity_value = static_cast<std::uint16_t>(message->data) % 1000;

    auto linear_velocity = m_driver_client.holding_registers(0x0001_addr, 2);
    auto error = (linear_velocity = {0, velocity_value});

    RCLCPP_INFO(get_logger(), "Error status: %s", error.message());
  }
}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::motor_control)
