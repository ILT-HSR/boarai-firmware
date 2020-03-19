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
    if (!m_angular_velocity_limit || limit < m_angular_velocity_limit)
    {
      log_info("received new angular velocity limit from {}: {}", source, limit);
      m_angular_velocity_limit = limit;
    }
  }

  auto mode_controller::on_linear_velocity_limit_update(std::string source, messages::LinearVelocity::SharedPtr new_limit)
      -> void
  {
    auto limit = new_limit->value;
    if (!m_linear_velocity_limit || limit < m_linear_velocity_limit)
    {
      log_info("received new linear velocity limit from {}: {}", source, limit);
      m_linear_velocity_limit = limit;
    }
  }

  auto mode_controller::on_gamepad_input_update(interface::topic::gamepad_input_t::SharedPtr new_input) -> void
  {
    auto [throttle, steering] = *new_input;
    log_debug("received new gamepad input: throttle: {} || steering: {}", throttle, steering);
    auto real_throttle{throttle * *m_linear_velocity_limit};
    auto real_steering{steering * *m_angular_velocity_limit};

    if (m_set_target_velocity_client->service_is_ready())
    {
      log_debug("calling '{}' with throttle=={} and steering=={}",
                m_set_target_velocity_client->get_service_name(),
                real_throttle,
                real_steering);
      auto request = std::make_shared<control::service::set_target_velocity_t::Request>();
      request->velocity =
          messages::PolarVelocity{}.set__value(messages::Polar2D{}.set__r(real_throttle).set__phi(real_steering));
      m_set_target_velocity_client->async_send_request(request);
    }
  }
}  // namespace boarai::intelligence