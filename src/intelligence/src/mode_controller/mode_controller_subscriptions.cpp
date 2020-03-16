#include "mode_controller/mode_controller.hpp"
#include "support/interfaces.hpp"
#include "support/messages.hpp"

#include <functional>
#include <string>

using namespace std::placeholders;

namespace boarai::intelligence
{
  auto mode_controller::subscribe_to_limit_topic(std::string name, std::string type) -> void
  {
    if (type == "boarai_support/msg/AngularVelocity")
    {
      std::function<void(messages::AngularVelocity::SharedPtr)> callback =
          std::bind(&mode_controller::on_agular_velocity_limit_update, this, name, _1);
      m_limit_subscriptions[name] = create_subscription<messages::AngularVelocity>(name, default_limit_policy, callback);
    }
    else if (type == "boarai_support/msg/LinearVelocity")
    {
      std::function<void(messages::LinearVelocity::SharedPtr)> callback =
          std::bind(&mode_controller::on_linear_velocity_limit_update, this, name, _1);
      m_limit_subscriptions[name] = create_subscription<messages::LinearVelocity>(name, default_limit_policy, callback);
    }
    else
    {
      log_warning("unknown limit type: {}", type);
    }
  }

  auto mode_controller::on_agular_velocity_limit_update(std::string source, messages::AngularVelocity::SharedPtr new_limit)
      -> void
  {
    auto limit = new_limit->value;
    if (limit < m_angular_velocity_limit)
    {
      log_info("received new angular velocity limit from {}: {}", source, limit);
      m_angular_velocity_limit = limit;
    }
  }

  auto mode_controller::on_linear_velocity_limit_update(std::string source, messages::LinearVelocity::SharedPtr new_limit)
      -> void
  {
    auto limit = new_limit->value;
    if (limit < m_linear_velocity_limit)
    {
      log_info("received new linear velocity limit from {}: {}", source, limit);
      m_linear_velocity_limit = limit;
    }
  }
}  // namespace boarai::intelligence