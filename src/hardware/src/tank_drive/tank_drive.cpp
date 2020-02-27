#include "tank_drive/tank_drive.hpp"

#include "hardware/layer_interface.hpp"
#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "roboteq/channel.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/to_string.hpp"

#include <modbuscpp/address.hpp>
#include <modbuscpp/context.hpp>
#include <modbuscpp/tcp_context.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <string>

using namespace std::chrono_literals;
using namespace modbus::modbus_literals;
using namespace std::placeholders;

auto constexpr node_name{"tank_drive"};

auto static make_context(std::string address, std::uint16_t port) -> modbus::context
{
  auto tcp_context = modbus::tcp_context{address, port};
  tcp_context.slave_id(1);
  tcp_context.response_timeout(1s);

  return std::move(tcp_context);
}

namespace boarai::hardware
{

  tank_drive::tank_drive(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}

  {
    declare_parameters();

    if (driver_enabled())
    {
      initialize_driver(driver_address(), driver_port());
      if (!m_motor_driver)
      {
        set_parameter(rclcpp::Parameter{to_string(parameter::driver_enabled), false});
      }
      else
      {
        m_motor_driver->set_motor_command(roboteq::channel::velocity, 0);
        m_motor_driver->set_motor_command(roboteq::channel::steering, 0);
        start_services();
        start_publishers();
        start_timers();
      }
    }

    m_parameter_change_handler = add_on_set_parameters_callback(std::bind(&tank_drive::on_parameters_changed, this, _1));
  }

  auto tank_drive::start_publishers() -> void
  {
    m_battery_voltages_publisher = create_publisher<topic::battery_voltage_t>(topic::battery_voltage, 10);
    m_drive_velocity_publisher = create_publisher<topic::drive_velocity_t>(topic::drive_velocity, 10);
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

  auto tank_drive::maximum_angular_velocity() -> double
  {
    assert(wheel_spacing() > 0.0);
    auto turn_circumference = wheel_spacing() * M_PI;
    return 360.0 * maximum_linear_velocity() / turn_circumference;
  }

}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::tank_drive)
