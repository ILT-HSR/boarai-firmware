#include "support/interfaces.hpp"
#include "support/lerp.hpp"
#include "tank_drive_debug/tank_drive_debug.hpp"

#include <chrono>
#include <functional>
#include <mutex>

namespace boarai::hardware
{
  using namespace std::chrono_literals;

  auto tank_drive_debug::start_timers() -> void
  {
    m_drive_velocity_update_timer =
        create_wall_timer(100ms, std::bind(&tank_drive_debug::on_drive_velocity_update_timer_expired, this));
  }

  auto tank_drive_debug::on_drive_velocity_update_timer_expired() -> void
  {
    if (m_requested_velocity)
    {
      auto lock = std::unique_lock{m_command_mutex};
      m_current_velocity.value.r = lerp(m_current_velocity.value.r, m_requested_velocity->value.r, acceleration_delay());
      m_current_velocity.value.phi = lerp(m_current_velocity.value.phi, m_requested_velocity->value.phi, acceleration_delay());
      lock.unlock();
    }

    auto msg = topic::drive_velocity_t{};

    msg.value.r = m_current_velocity.value.r;
    msg.value.phi = m_current_velocity.value.phi;

    m_drive_velocity_publisher->publish(msg);
  }

}  // namespace boarai::hardware