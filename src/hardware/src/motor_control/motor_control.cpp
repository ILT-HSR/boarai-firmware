#include "motor_control/motor_control.hpp"

#include "layer_constants.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "roboteq/channel.hpp"
#include "roboteq/driver.hpp"
#include "support/enum_utility.hpp"
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

  motor_control::motor_control(rclcpp::NodeOptions const & options)
      : Node{MOTOR_CONTROL_NODE_NAME, LAYER_NAMESPACE, options}
  {
    declare_parameters();
    initialize_driver();
  }

  auto motor_control::declare_parameters() -> void
  {
    declare_parameter(to_string(parameter::driver_address), DEFAULT_DRIVER_ADDRESS);
    declare_parameter(to_string(parameter::driver_port), DEFAULT_DRIVER_PORT);
    declare_parameter(to_string(parameter::driver_enabled), DEFAULT_DRIVER_ENABLED);
  }

  auto motor_control::initialize_driver() -> void
  {
    auto const [enable_driver, address, port] = [&] {
      auto parameters = get_parameters({
          to_string(parameter::driver_enabled),
          to_string(parameter::driver_address),
          to_string(parameter::driver_port),
      });
      return std::tuple{
          parameters.at(0).as_bool(),
          parameters.at(1).as_string(),
          parameters.at(2).as_int(),
      };
    }();

    if (!enable_driver)
    {
      RCLCPP_INFO(get_logger(), "driver connections has been disabled. Skipping driver initialization.");
      return;
    }

    RCLCPP_INFO(get_logger(), "connecting to motor driver at '%s:%d'", address.c_str(), port);

    try
    {
      m_driver_connection.emplace(make_context(address, port));
    }
    catch (std::exception const & e)
    {
      RCLCPP_ERROR(get_logger(), "failed to connect to driver: %s", e.what());
    }
  }

  auto motor_control::handle_message(std_msgs::msg::Float32::SharedPtr message) -> void
  {
    static_cast<void>(message);
  }
}  // namespace boarai::hardware

namespace boarai
{
  using namespace hardware;

  auto constexpr parameter_names = std::array{
      std::pair{motor_control::parameter::driver_address, "driver_address"},
      std::pair{motor_control::parameter::driver_port, "driver_port"},
      std::pair{motor_control::parameter::driver_enabled, "driver_enabled"},
  };

  static_assert(enum_mappings_are_unique(parameter_names), "missing mapping for parameter");
  static_assert(enum_map_has_all_entries(parameter_names, motor_control::parameter::driver_address),
                "duplicate key or value in parameter mappings");

  template<>
  auto to_string(motor_control::parameter const & object) -> std::string
  {
    assert(is_valid<motor_control::parameter>(static_cast<std::underlying_type_t<motor_control::parameter>>(object)));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.first == object;
    });

    return found->second;
  }

  template<>
  auto from_string(std::string const & stringified) -> hardware::motor_control::parameter
  {
    assert(is_valid<motor_control::parameter>(stringified));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.second == stringified;
    });

    return found->first;
  }

  template<>
  auto is_valid<hardware::motor_control::parameter>(std::underlying_type_t<hardware::motor_control::parameter> candidate)
      -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }

  template<>
  auto is_valid<hardware::motor_control::parameter>(std::string const & candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }
}  // namespace boarai

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::motor_control)
