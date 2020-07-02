#include "imu/imu.hpp"
#include "support/interfaces.hpp"

#include <chrono>
#include <functional>

using namespace std::chrono_literals;

namespace boarai::hardware
{

  auto imu::start_timers() -> void
  {
    m_orientation_update_timer = create_wall_timer(100ms, std::bind(&imu::on_orientation_update_timer_expired, this));
  }

  auto imu::on_orientation_update_timer_expired() -> void
  {
    auto orientation = m_device->euler_orientation();
    auto message = topic::imu_orientation_t{};
    message.yaw = orientation.heading / 16.0;
    message.pitch = orientation.pitch / 16.0;
    message.roll = orientation.roll / 16.0;
    m_orientation_publisher->publish(message);
  }

}  // namespace boarai::hardware