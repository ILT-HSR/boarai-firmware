#include "velocity_control/velocity_control.hpp"

#include <chrono>
#include <functional>

namespace boarai::control
{
  using namespace std::chrono_literals;

  auto velocity_control::start_timers() -> void
  {
    m_velocity_update_timer = create_wall_timer(200ms, std::bind(&velocity_control::on_velocity_update_timer_expired, this));
  }

  auto velocity_control::on_velocity_update_timer_expired() -> void
  {
    if (m_drive_velocity_client->service_is_ready())
    {
      auto [currentVelocityLinear, currentVelocityAngular] = m_velocity_current->value;
      auto [targetVelocityLinear, targetVelocityAngular] = m_velocity_target->value;

      auto linearCorrection = targetVelocityLinear + m_controller_proportional(currentVelocityLinear, targetVelocityLinear);
      auto angularCorrection = targetVelocityAngular + m_controller_proportional(currentVelocityAngular, targetVelocityAngular);

      boarai_support::msg::Polar2D velocity{};
      velocity.set__r(linearCorrection);
      velocity.set__phi(angularCorrection);

      log_debug("calling '{}' with linear=={} and angular=={}",
                m_drive_velocity_client->get_service_name(),
                velocity.r,
                velocity.phi);
      auto hw_request = std::make_shared<hardware::service::set_drive_velocity_t::Request>();
      hw_request->velocity = messages::PolarVelocity{}.set__value(velocity);
      m_drive_velocity_client->async_send_request(hw_request);
    }
  }
}  // namespace boarai::control