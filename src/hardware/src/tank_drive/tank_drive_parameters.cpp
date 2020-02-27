#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/parameter.hpp"
#include "support/enum_utility.hpp"
#include "support/to_string.hpp"
#include "tank_drive/tank_drive.hpp"

#include <cstdint>
#include <string>
#include <vector>

using namespace std::string_literals;

auto const DEFAULT_DRIVER_ADDRESS{"192.168.1.20"s};
auto const DEFAULT_DRIVER_PORT{static_cast<std::uint16_t>(502)};
auto const DEFAULT_DRIVER_ENABLED{true};
auto const DEFAULT_WHEEL_SPACING{0.0};
auto const DEFAULT_MAXIMUM_LINEAR_VELOCITY{1.0};

namespace boarai::hardware
{

  auto tank_drive::declare_parameters() -> void
  {
    declare_parameter(to_string(parameter::driver_address), DEFAULT_DRIVER_ADDRESS);
    declare_parameter(to_string(parameter::driver_port), DEFAULT_DRIVER_PORT);
    declare_parameter(to_string(parameter::driver_enabled), DEFAULT_DRIVER_ENABLED);
    declare_parameter(to_string(parameter::wheel_spacing), DEFAULT_WHEEL_SPACING);
    declare_parameter(to_string(parameter::maximum_linear_velocity), DEFAULT_MAXIMUM_LINEAR_VELOCITY);
  }

  auto tank_drive::driver_enabled() -> bool
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

  auto tank_drive::driver_port() -> std::uint16_t
  {
    auto result = std::uint16_t{};
    get_parameter_or(to_string(parameter::driver_port), result, DEFAULT_DRIVER_PORT);
    return result;
  }

  auto tank_drive::wheel_spacing() -> double
  {
    auto result{0.0};
    get_parameter_or(to_string(parameter::wheel_spacing), result, DEFAULT_WHEEL_SPACING);
    return result;
  }

  auto tank_drive::maximum_linear_velocity() -> double
  {
    auto result{0.0};
    get_parameter_or(to_string(parameter::maximum_linear_velocity), result, DEFAULT_MAXIMUM_LINEAR_VELOCITY);
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
        if (driver_enabled() != param.as_bool())
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
      case parameter::wheel_spacing:
      case parameter::maximum_linear_velocity:
        result.successful &= param.as_double() > 0.0;
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
    if (driver_enabled() && m_motor_driver)
    {
      disconnect_driver();
    }
    if (!new_value.empty() && driver_enabled())
    {
      initialize_driver(new_value, driver_port());
      return static_cast<bool>(m_motor_driver);
    }
    return true;
  }

  auto tank_drive::on_driver_port_changed(std::int64_t new_value) -> bool
  {
    if (driver_enabled() && m_motor_driver)
    {
      disconnect_driver();
    }
    if (new_value && driver_enabled())
    {
      initialize_driver(driver_address(), new_value);
      return static_cast<bool>(m_motor_driver);
    }

    return true;
  }

}  // namespace boarai::hardware