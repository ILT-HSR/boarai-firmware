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
              on_voltages_updated(value);
            }
          },
          m_motor_driver->read_volts_battery());
    }
  }

}  // namespace boarai::hardware