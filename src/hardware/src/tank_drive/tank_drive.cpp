#include "tank_drive/tank_drive.hpp"

#include "layer_constants.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "roboteq/channel.hpp"
#include "roboteq/driver.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/messages.hpp"
#include "support/services.hpp"
#include "support/to_string.hpp"

#include <modbuscpp/address.hpp>
#include <modbuscpp/context.hpp>

#include <algorithm>
#include <array>
#include <cassert>
#include <chrono>
#include <cstdint>
#include <functional>
#include <iterator>
#include <string>
#include <utility>

using namespace std::chrono_literals;
using namespace std::string_literals;
using namespace std::placeholders;
using namespace modbus::modbus_literals;

auto constexpr DEFAULT_DRIVER_ADDRESS{"192.168.1.20"};
auto constexpr DEFAULT_DRIVER_PORT{502};
auto constexpr DEFAULT_DRIVER_ENABLED{true};

namespace boarai::hardware
{
  namespace
  {
    auto make_context(std::string address, std::uint16_t port) -> modbus::context
    {
      auto tcp_context = modbus::tcp_context{address, port};
      tcp_context.slave_id(1);
      tcp_context.response_timeout(1s);

      return std::move(tcp_context);
    }
  }  // namespace

  tank_drive::tank_drive(rclcpp::NodeOptions const & options)
      : fmt_node{TANK_DRIVE_NODE_NAME, LAYER_NAMESPACE, options}

  {
    declare_parameters();
    if (is_driver_enabled())
    {
      initialize_driver(driver_address(), driver_port());
      if (!m_driver_connection)
      {
        set_parameter(rclcpp::Parameter{to_string(parameter::driver_enabled), false});
      }
    }

    m_drive_velocity_service =
        create_service<services::SetDriveVelocity>(HARDWARE_SERVICE_SET_DRIVE_VELOCITY,
                                                   std::bind(&tank_drive::on_set_drive_velocity_request, this, _1, _2));
    m_on_parameters_changed_handler = add_on_set_parameters_callback(std::bind(&tank_drive::on_parameters_changed, this, _1));
  }

  auto tank_drive::declare_parameters() -> void
  {
    declare_parameter(to_string(parameter::driver_address), DEFAULT_DRIVER_ADDRESS);
    declare_parameter(to_string(parameter::driver_port), DEFAULT_DRIVER_PORT);
    declare_parameter(to_string(parameter::driver_enabled), DEFAULT_DRIVER_ENABLED);
  }

  auto tank_drive::initialize_driver(std::string address, std::uint16_t port) -> void
  {
    assert(!m_motor_driver);

    try
    {
      log_info("connecting to motor driver at '{}:{}'", address, port);
      m_driver_connection.emplace(make_context(address, port));
      m_driver_client.emplace(*m_driver_connection);
      m_motor_driver.emplace(*m_driver_client);
    }
    catch (std::exception const & e)
    {
      log_error("failed to connect to driver: {}", e.what());
    }
  }

  auto tank_drive::disconnect_driver() -> void
  {
    if (m_motor_driver)
    {
      log_info("disconnecting from motor driver");
      m_motor_driver.reset();
      m_driver_client.reset();
      m_driver_connection.reset();
    }
  }

  auto tank_drive::is_driver_enabled() -> bool
  {
    auto result{false};
    get_parameter_or(to_string(parameter::driver_enabled), result, DEFAULT_DRIVER_ENABLED);
    return result;
  }

  auto tank_drive::driver_address() -> std::string
  {
    auto result{""s};
    get_parameter_or(to_string(parameter::driver_address), result, std::string{DEFAULT_DRIVER_ADDRESS});
    return result;
  }

  auto tank_drive::driver_port() -> std::int64_t
  {
    auto result = std::int64_t{};
    get_parameter_or(to_string(parameter::driver_port), result, static_cast<std::int64_t>(DEFAULT_DRIVER_PORT));
    return result;
  }

  auto tank_drive::on_parameters_changed(std::vector<rclcpp::Parameter> new_parameters)
      -> rcl_interfaces::msg::SetParametersResult
  {
    auto result = rcl_interfaces::msg::SetParametersResult{};
    result.successful = true;

    for (auto & param : new_parameters)
    {
      auto name = param.get_name();

      if (!is_valid<tank_drive::parameter>(name))
      {
        log_warning("received change of unknown parameter '{}'", name);
        result.successful = false;
        return result;
      }

      switch (from_string<tank_drive::parameter>(name))
      {
      case parameter::driver_enabled:
        if (is_driver_enabled() != param.as_bool())
        {
          result.successful &= on_driver_enabled_changed(param.as_bool());
        }
        break;
      case parameter::driver_address:
        if (driver_address() != param.as_string())
        {
          result.successful &= on_driver_address_changed(param.as_string());
        }
        break;
      case parameter::driver_port:
        if (driver_port() != param.as_int())
        {
          result.successful &= on_driver_port_changed(param.as_int());
        }
        break;
      case parameter::END_OF_ENUM:
        break;
      };
    }

    return result;
  }

  auto tank_drive::on_driver_enabled_changed(bool new_value) -> bool
  {
    if (!new_value && m_motor_driver)
    {
      disconnect_driver();
      return true;
    }
    else if (new_value && !m_motor_driver)
    {
      initialize_driver(driver_address(), driver_port());
      return static_cast<bool>(m_motor_driver);
    }
    return true;
  }

  auto tank_drive::on_driver_address_changed(std::string new_value) -> bool
  {
    if (is_driver_enabled() && m_motor_driver)
    {
      disconnect_driver();
    }
    if (!new_value.empty() && is_driver_enabled())
    {
      initialize_driver(new_value, driver_port());
      return static_cast<bool>(m_motor_driver);
    }
    return true;
  }

  auto tank_drive::on_driver_port_changed(std::int64_t new_value) -> bool
  {
    if (is_driver_enabled() && m_motor_driver)
    {
      disconnect_driver();
    }
    if (new_value && is_driver_enabled())
    {
      initialize_driver(driver_address(), new_value);
      return static_cast<bool>(m_motor_driver);
    }

    return true;
  }

  auto tank_drive::on_set_drive_velocity_request(std::shared_ptr<services::SetDriveVelocity::Request> request,
                                                 std::shared_ptr<services::SetDriveVelocity::Response>) -> void
  {
    auto velocity = request->velocity;
    log_info("received request to set velocity to: r={} and phi={}", velocity.r, velocity.phi);
  }

}  // namespace boarai::hardware

namespace boarai
{
  using namespace hardware;

  auto constexpr parameter_names = std::array{
      std::pair{tank_drive::parameter::driver_address, "driver_address"},
      std::pair{tank_drive::parameter::driver_port, "driver_port"},
      std::pair{tank_drive::parameter::driver_enabled, "driver_enabled"},
  };

  static_assert(enum_mappings_are_unique(parameter_names), "missing mapping for parameter");
  static_assert(enum_map_has_all_entries(parameter_names, tank_drive::parameter::driver_address),
                "duplicate key or value in parameter mappings");

  template<>
  auto to_string(tank_drive::parameter const & object) -> std::string
  {
    assert(is_valid<tank_drive::parameter>(static_cast<std::underlying_type_t<tank_drive::parameter>>(object)));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.first == object;
    });

    return found->second;
  }

  template<>
  auto from_string(std::string const & stringified) -> hardware::tank_drive::parameter
  {
    assert(is_valid<tank_drive::parameter>(stringified));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.second == stringified;
    });

    return found->first;
  }

  template<>
  auto is_valid<hardware::tank_drive::parameter>(std::underlying_type_t<hardware::tank_drive::parameter> candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }

  template<>
  auto is_valid<hardware::tank_drive::parameter>(std::string const & candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }
}  // namespace boarai

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::tank_drive)
