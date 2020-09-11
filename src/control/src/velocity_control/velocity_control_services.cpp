#include "velocity_control/velocity_control.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::control
{
  auto velocity_control::start_services() -> void
  {
    m_set_target_velocity_service = create_service<service::set_target_velocity_t>(
        service::set_target_velocity,
        std::bind(&velocity_control::on_set_target_velocity_request, this, _1, _2));
  }

  auto velocity_control::on_set_target_velocity_request(service::set_target_velocity_t::Request::SharedPtr request,
                                                        service::set_target_velocity_t::Response::SharedPtr) -> void
  {
    m_velocity_target = request->velocity;
    log_info("received request to set velocity to: linear: {} || angular: {}",
             m_velocity_target->value.r,
             m_velocity_target->value.phi);
  }
}  // namespace boarai::control