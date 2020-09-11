#include "support/interfaces.hpp"
#include "support/string_utility.hpp"
#include "velocity_control/velocity_control.hpp"

#include <functional>

namespace boarai::control
{
  using namespace std::placeholders;

  auto velocity_control::start_subscriptions() -> void
  {
    m_estimated_velocity_subscription = create_subscription<estimation::topic::estimated_velocity_t>(
        join("/", estimation::ros_namespace, estimation::topic::estimated_velocity),
        default_topic_policy,
        std::bind(&velocity_control::on_estimated_velocity_update, this, _1));
  }

  auto velocity_control::on_estimated_velocity_update(estimation::topic::estimated_velocity_t::SharedPtr new_velocity) -> void
  {
    m_velocity_current = *new_velocity;
    log_info("new estimated velocity: linear = {:.2f} || angular = {:.2f}",
             m_velocity_current->value.r,
             m_velocity_current->value.phi);
  }

}  // namespace boarai::control