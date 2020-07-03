#include "position_estimator/position_estimator.hpp"
#include "support/interfaces.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::estimation
{

  auto position_estimator::start_services() -> void
  {
    m_get_estimated_position_service = create_service<service::get_estimated_position_t>(
        service::get_estimated_position,
        std::bind(&position_estimator::on_estimated_position_request, this, _1, _2));
  }

  auto position_estimator::on_estimated_position_request(service::get_estimated_position_t::Request::SharedPtr,
                                                         service::get_estimated_position_t::Response::SharedPtr response)
      -> void
  {
    response->success = m_last_global_position && m_last_imu_orientation;
    if (response->success)
    {
      response->position = *m_last_global_position;
      response->orientation = m_last_imu_orientation->orientation;
    }
  }

}  // namespace boarai::estimation