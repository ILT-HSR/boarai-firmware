#include "imu/imu.hpp"
#include "support/interfaces.hpp"

#include <chrono>
#include <functional>
#include <stdexcept>

using namespace std::chrono_literals;

namespace boarai::hardware
{

  auto imu::start_timers() -> void
  {
    m_orientation_update_timer = create_wall_timer(100ms, std::bind(&imu::on_orientation_update_timer_expired, this));
  }

  auto imu::on_orientation_update_timer_expired() -> void
  {
    if (!m_time_source_initialized)
    {
      m_time_source.attachNode(shared_from_this());
      m_time_source.attachClock(m_clock);
      m_time_source_initialized = true;
    }

    try
    {
      auto orientation = m_device->euler_orientation();
      auto message = topic::imu_orientation_t{};
      message.header.frame_id = "vehicle";
      message.header.stamp = m_clock->now();
      message.yaw = orientation.heading / 16.0;
      message.pitch = orientation.pitch / 16.0;
      message.roll = orientation.roll / 16.0;
      m_orientation_publisher->publish(message);
    }
    catch (std::exception const & e)
    {
      log_error("Failed to read IMU! reason: {}", e.what());
    }
  }

}  // namespace boarai::hardware