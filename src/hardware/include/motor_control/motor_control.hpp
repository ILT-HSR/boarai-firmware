#ifndef BOARAI_HARDWARE_MOTOR_CONTROL_HPP
#define BOARAI_HARDWARE_MOTOR_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "roboteq/driver.hpp"
#include "std_msgs/msg/float32.hpp"
#include "support/to_string.hpp"

#include <modbuscpp/client.hpp>
#include <modbuscpp/connection.hpp>
#include <modbuscpp/context.hpp>
#include <modbuscpp/tcp_context.hpp>

#include <optional>
#include <string>

namespace boarai::hardware
{

  auto constexpr MOTOR_CONTROL_NODE_NAME{"motor_control"};

  struct motor_control : rclcpp::Node
  {
    using super = rclcpp::Node;
    using super::declare_parameters;

    enum struct parameter
    {
      driver_address,
      driver_port,
      driver_enabled,
    };

    explicit motor_control(rclcpp::NodeOptions const & options);

  private:
    auto declare_parameters() -> void;
    auto initialize_driver() -> void;

    auto handle_message(std_msgs::msg::Float32::SharedPtr message) -> void;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subscription;
    std::optional<modbus::connection> m_driver_connection{};
    std::optional<modbus::client> m_driver_client{};
    std::optional<roboteq::driver> m_motor_driver{};
  };

}  // namespace boarai::hardware

namespace boarai
{

  template<>
  auto to_string(hardware::motor_control::parameter const & object) -> std::string;

  template<>
  auto from_string(std::string const & stringified) -> hardware::motor_control::parameter;

}  // namespace boarai

#endif