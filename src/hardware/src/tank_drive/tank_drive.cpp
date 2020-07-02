#include "tank_drive/tank_drive.hpp"

#include "rcl_interfaces/msg/parameter_descriptor.hpp"
#include "rcl_interfaces/msg/parameter_type.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "roboteq/channel.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"
#include "support/string_utility.hpp"
#include "support/to_string.hpp"

#include <modbuscpp/address.hpp>
#include <modbuscpp/context.hpp>
#include <modbuscpp/tcp_context.hpp>

#include <cassert>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

using namespace std::chrono_literals;
using namespace modbus::modbus_literals;
using namespace std::placeholders;

auto static make_context(std::string address, std::uint16_t port) -> modbus::context
{
  auto tcp_context = modbus::tcp_context{address, port};
  tcp_context.slave_id(1);
  tcp_context.response_timeout(1s);

  return std::move(tcp_context);
}

namespace boarai::hardware
{

  namespace
  {
    auto constexpr node_name{"tank_drive"};

    auto const angular_velocity_limit_topic{join("/", ros_limit_namespace, join("_", limit::angular_velocity, node_name))};
    auto const linear_velocity_limit_topic{join("/", ros_limit_namespace, join("_", limit::linear_velocity, node_name))};
  }  // namespace

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
        // TODO: reenable when the hardware driver interface is not unusably slow anymore
        // start_timers();
        start_driver_worker();
      }

      publish_limits();
    }

    m_parameter_change_handler = add_on_set_parameters_callback(std::bind(&tank_drive::on_parameters_changed, this, _1));
  }

  tank_drive::~tank_drive()
  {
    m_run_driver_worker = false;
    m_driver_worker.join();
  }

  auto tank_drive::start_driver_worker() -> void
  {
    m_run_driver_worker = true;
    m_driver_worker = std::thread([&] {
      while (m_run_driver_worker)
      {
        auto lock = std::unique_lock{m_command_mutex};
        auto request = m_requested_velocity;
        m_requested_velocity.reset();
        lock.unlock();

        if (request && m_motor_driver)
        {
          process_driver_command(*request);
        }
      }
    });
  }

  auto tank_drive::process_driver_command(boarai::messages::PolarVelocity velocity) -> void
  {
    auto [linear_velocity, angular_velocity] = velocity.value;

    auto throttle = static_cast<std::int32_t>(1000 / maximum_linear_velocity() * linear_velocity);
    throttle = throttle < 0 ? std::max(throttle, -1000) : std::min(throttle, 1000);
    log_info("determined throttle to be: {}", throttle);

    auto steering = static_cast<std::int32_t>(1000 / maximum_angular_velocity() * angular_velocity);
    steering = steering < 0 ? std::max(steering, -1000) : std::min(steering, 1000);
    log_info("determined steering to be: {}", steering);

    m_motor_driver->set_motor_command(roboteq::channel::velocity, throttle);
    m_motor_driver->set_motor_command(roboteq::channel::steering, steering);
  }

  auto tank_drive::publish_limits() -> void
  {
    m_angular_velocity_limit_publisher =
        create_publisher<limit::angular_velocity_t>(angular_velocity_limit_topic, default_limit_policy);
    m_angular_velocity_limit_publisher->publish(messages::AngularVelocity{}.set__value(maximum_angular_velocity()));

    m_linear_velocity_limit_publisher =
        create_publisher<limit::linear_velocity_t>(linear_velocity_limit_topic, default_limit_policy);
    m_linear_velocity_limit_publisher->publish(messages::LinearVelocity{}.set__value(maximum_linear_velocity()));
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
