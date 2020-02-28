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
        10,
        std::bind(&velocity_control::on_estimated_velocity_update, this, _1));
  }

  auto velocity_control::on_estimated_velocity_update(estimation::topic::estimated_velocity_t::SharedPtr new_velocity) -> void
  {
    auto [linear_velocity, angular_velocity] = new_velocity->value;
    log_info("new estimated velocity: linear = {:.2f} || angular = {:.2f}", linear_velocity, angular_velocity);
  }

}  // namespace boarai::control