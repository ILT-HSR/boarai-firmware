#include "roboteq/channel.hpp"
#include "support/interfaces.hpp"
#include "support/services.hpp"
#include "tank_drive/tank_drive.hpp"

#include <cmath>
#include <functional>
#include <future>
#include <mutex>
#include <optional>

using namespace std::placeholders;

namespace boarai::hardware
{

  auto tank_drive::start_services() -> void
  {
    // clang-format off
    m_drive_velocity_service = create_service<service::set_drive_velocity_t>(
        service::set_drive_velocity,
        std::bind(&tank_drive::on_drive_velocity_request, this, _1, _2));
    m_angular_velocity_service = create_service<service::get_maximum_angular_velocity_t>(
        service::get_maximum_angular_velocity,
        std::bind(&tank_drive::on_angular_velocity_request, this, _1, _2));
    // clang-format on
  }  // namespace boarai::hardware

  auto tank_drive::on_drive_velocity_request(service::set_drive_velocity_t::Request::SharedPtr request,
                                             service::set_drive_velocity_t::Response::SharedPtr) -> void
  {
    auto guard = std::lock_guard{m_command_mutex};
    m_requested_velocity = request->velocity;
  }

  auto tank_drive::on_angular_velocity_request(service::get_maximum_angular_velocity_t::Request::SharedPtr,
                                               service::get_maximum_angular_velocity_t::Response::SharedPtr response) -> void
  {
    response->velocity = maximum_angular_velocity();
  }

}  // namespace boarai::hardware