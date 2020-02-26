#ifndef BOARAI_HARDWARE_MOTOR_CONTROL_HPP
#define BOARAI_HARDWARE_MOTOR_CONTROL_HPP

#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/rclcpp.hpp"
#include "roboteq/driver.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/messages.hpp"
#include "support/services.hpp"
#include "support/to_string.hpp"

#include <modbuscpp/client.hpp>
#include <modbuscpp/connection.hpp>
#include <modbuscpp/context.hpp>
#include <modbuscpp/tcp_context.hpp>

#include <cstdint>
#include <optional>
#include <string>
#include <type_traits>
#include <utility>

namespace boarai::hardware
{

  auto constexpr TANK_DRIVE_NODE_NAME{"tank_drive"};

  struct tank_drive : fmt_node
  {
    using super = rclcpp::Node;
    using super::declare_parameter;
    using super::declare_parameters;

    enum struct parameter
    {
      driver_address,
      driver_port,
      driver_enabled,
      wheel_spacing,
      maximum_linear_velocity,

      // End Marker for enum_utility support
      END_OF_ENUM
    };

    explicit tank_drive(rclcpp::NodeOptions const & options);

  private:
    auto declare_parameters() -> void;
    auto initialize_driver(std::string address, std::uint16_t port) -> void;
    auto disconnect_driver() -> void;

    auto is_driver_enabled() -> bool;
    auto driver_address() -> std::string;
    auto driver_port() -> std::uint16_t;
    auto wheel_spacing() -> double;
    auto maximum_linear_velocity() -> double;
    auto maximum_angular_velocity() -> double;

    auto on_parameters_changed(std::vector<rclcpp::Parameter> new_parameters) -> rcl_interfaces::msg::SetParametersResult;
    auto on_driver_enabled_changed(bool new_value) -> bool;
    auto on_driver_address_changed(std::string new_value) -> bool;
    auto on_driver_port_changed(std::int64_t new_value) -> bool;
    auto on_wheel_spacing_changed(double new_value) -> bool;

    auto on_set_drive_velocity_request(std::shared_ptr<services::SetDriveVelocity::Request> request,
                                       std::shared_ptr<services::SetDriveVelocity::Response> response) -> void;

    rclcpp::Service<services::SetDriveVelocity>::SharedPtr m_drive_velocity_service;
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
  auto to_string(hardware::tank_drive::parameter const & object) -> std::string;

  template<>
  auto from_string(std::string const & stringified) -> hardware::tank_drive::parameter;

  template<>
  auto is_valid<hardware::tank_drive::parameter>(std::underlying_type_t<hardware::tank_drive::parameter> candidate) -> bool;

  template<>
  auto is_valid<hardware::tank_drive::parameter>(std::string const & candidate) -> bool;

}  // namespace boarai

#endif