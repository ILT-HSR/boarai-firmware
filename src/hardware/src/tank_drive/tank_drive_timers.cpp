#include "tank_drive/tank_drive.hpp"

#include <chrono>
#include <functional>
#include <system_error>
#include <type_traits>
#include <variant>

namespace boarai::hardware
{

  using namespace std::chrono_literals;

  auto tank_drive::start_timers() -> void
  {
    m_voltages_update_timer = create_wall_timer(1s, std::bind(&tank_drive::on_voltages_update_timer_expired, this));
    m_drive_velocity_update_timer =
        create_wall_timer(100ms, std::bind(&tank_drive::on_drive_velocity_update_timer_expired, this));
  }

  auto tank_drive::on_voltages_update_timer_expired() -> void
  {
    if (m_motor_driver)
    {
      std::visit(
          [this](auto value) {
            if constexpr (std::is_same_v<std::error_code, decltype(value)>)
            {
              log_error("failed to read battery voltage: {}", value.message());
            }
            else
            {
              auto msg = messages::Voltage{};
              msg.volts = value / 10.0f;
              m_battery_voltages_publisher->publish(msg);
            }
          },
          m_motor_driver->read_volts_battery());
    }
  }

  auto tank_drive::on_drive_velocity_update_timer_expired() -> void
  {
    if (m_motor_driver)
    {
      auto channel_one = m_motor_driver->read_brushless_motor_speed(roboteq::channel::_1);
      auto channel_two = m_motor_driver->read_brushless_motor_speed(roboteq::channel::_2);

      if (!channel_one)
      {
        log_error("failed to read channel_one: {}", std::get<std::error_code>(channel_one).message());
      }
      else if (!channel_two)
      {
        log_error("failed to read channel_two: {}", std::get<std::error_code>(channel_two).message());
      }
      else
      {
        auto throttle_factor = (*channel_one + *channel_two) / 2;
        auto steering_factor = *channel_one - throttle_factor;
        auto msg = messages::PolarVelocity{};
        msg.value.r = maximum_linear_velocity() / 1000 * throttle_factor;
        msg.value.phi = maximum_angular_velocity() / 1000 * steering_factor;
        m_drive_velocity_publisher->publish(msg);
      }
    }
  }

}  // namespace boarai::hardware