#ifndef BOARAI_HARDWARE_MOTOR_CONTROL_HPP
#define BOARAI_HARDWARE_MOTOR_CONTROL_HPP

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboteq/driver.hpp"
#include "std_msgs/msg/float32.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/messages.hpp"
#include "support/to_string.hpp"

#include <modbuscpp/client.hpp>
#include <modbuscpp/connection.hpp>
#include <modbuscpp/context.hpp>
#include <modbuscpp/tcp_context.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>

namespace boarai::hardware
{

  auto constexpr MOTOR_CONTROL_NODE_NAME{"motor_control"};

  struct motor_control : fmt_node
  {
    using super = rclcpp::Node;
    using super::declare_parameters;

    enum struct parameter
    {
      driver_address,
      driver_port,
      driver_enabled,

      // End Marker for enum_utility support
      END_OF_ENUM
    };

    explicit motor_control(rclcpp::NodeOptions const & options);

  private:
    auto declare_parameters() -> void;
    auto initialize_driver(std::string address, std::uint16_t port) -> void;
    auto disconnect_driver() -> void;

    auto is_driver_enabled() -> bool;
    auto driver_address() -> std::string;
    auto driver_port() -> std::int64_t;

    auto on_parameters_changed(std::vector<rclcpp::Parameter> new_parameters) -> rcl_interfaces::msg::SetParametersResult;
    auto on_driver_enabled_changed(bool new_value) -> bool;
    auto on_driver_address_changed(std::string new_value) -> bool;
    auto on_driver_port_changed(std::int64_t new_value) -> bool;

    auto handle_message(messages::Polar2D::SharedPtr message) -> void;

    rclcpp::Subscription<messages::Polar2D>::SharedPtr m_subscription;
    OnSetParametersCallbackHandle::SharedPtr m_on_parameters_changed_handler;

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

  template<>
  auto is_valid<hardware::motor_control::parameter>(std::underlying_type_t<hardware::motor_control::parameter> candidate)
      -> bool;

  template<>
  auto is_valid<hardware::motor_control::parameter>(std::string const & candidate) -> bool;

}  // namespace boarai

#endif