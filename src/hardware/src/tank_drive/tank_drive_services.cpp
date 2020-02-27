#include "layer_constants.hpp"
#include "roboteq/channel.hpp"
#include "support/services.hpp"
#include "tank_drive/tank_drive.hpp"

#include <cmath>
#include <functional>

using namespace std::placeholders;

namespace boarai::hardware
{

  auto tank_drive::start_services() -> void
  {
    m_drive_velocity_service =
        create_service<services::SetDriveVelocity>(HARDWARE_SERVICE_SET_DRIVE_VELOCITY,
                                                   std::bind(&tank_drive::on_drive_velocity_request, this, _1, _2));
    m_angular_velocity_service =
        create_service<services::GetMaximumAngularVelocity>(HARDWARE_SERVICE_GET_MAXIMUM_ANGULAR_VELOCITY,
                                                            std::bind(&tank_drive::on_angular_velocity_request, this, _1, _2));
  }

  auto tank_drive::on_drive_velocity_request(services::SetDriveVelocity::Request::SharedPtr request,
                                             services::SetDriveVelocity::Response::SharedPtr) -> void
  {
    auto [linear_velocity, angular_velocity] = request->velocity.value;

    auto throttle = static_cast<std::int32_t>(1000 / maximum_linear_velocity() * linear_velocity);
    throttle = throttle < 0 ? std::max(throttle, -1000) : std::min(throttle, 1000);
    log_info("determined throttle to be: {}", throttle);

    auto steering = static_cast<std::int32_t>(1000 / maximum_angular_velocity() * angular_velocity);
    steering = steering < 0 ? std::max(steering, -1000) : std::min(steering, 1000);
    log_info("determined steering to be: {}", steering);

    if (m_motor_driver)
    {
      m_motor_driver->set_motor_command(roboteq::channel::velocity, throttle);
      m_motor_driver->set_motor_command(roboteq::channel::steering, steering);
    }
  }

  auto tank_drive::on_angular_velocity_request(services::GetMaximumAngularVelocity::Request::SharedPtr,
                                               services::GetMaximumAngularVelocity::Response::SharedPtr response) -> void
  {
    response->velocity = maximum_angular_velocity();
  }

}  // namespace boarai::hardware