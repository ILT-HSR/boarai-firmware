#include "position_estimator/position_estimator.hpp"
#include "support/interfaces.hpp"
#include "support/string_utility.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::estimation
{

  auto position_estimator::start_subscriptions() -> void
  {
    m_global_position_subscription = create_subscription<hardware::topic::global_position_t>(
        join("/", hardware::ros_namespace, hardware::topic::global_position),
        default_topic_policy,
        std::bind(&position_estimator::on_global_position_update, this, _1));

    m_imu_orientation_subscription = create_subscription<hardware::topic::imu_orientation_t>(
        join("/", hardware::ros_namespace, hardware::topic::imu_orientation),
        default_topic_policy,
        std::bind(&position_estimator::on_imu_orientation_update, this, _1));
  }

  auto position_estimator::on_global_position_update(hardware::topic::global_position_t::SharedPtr new_position) -> void
  {
    m_last_global_position = *new_position;
  }

  auto position_estimator::on_imu_orientation_update(hardware::topic::imu_orientation_t::SharedPtr new_orientation) -> void
  {
    m_last_imu_orientation = *new_orientation;
  }

}  // namespace boarai::estimation