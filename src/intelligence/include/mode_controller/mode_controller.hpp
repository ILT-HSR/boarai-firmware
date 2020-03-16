#ifndef BOARAI_INTELLIGENCE_MODE_CONTROLLER_HPP
#define BOARAI_INTELLIGENCE_MODE_CONTROLLER_HPP

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "support/fmt_node.hpp"
#include "support/messages.hpp"

#include <map>
#include <memory>
#include <optional>
#include <string>

namespace boarai::intelligence
{

  struct mode_controller : fmt_node
  {
    using super = rclcpp::Node;

    explicit mode_controller(rclcpp::NodeOptions const & options);

  private:
    auto start_timers() -> void;

    auto subscribe_to_limit_topic(std::string name, std::string type) -> void;

    auto on_limit_subscriber_timer_expired() -> void;

    auto on_agular_velocity_limit_update(std::string source, messages::AngularVelocity::SharedPtr new_limit) -> void;
    auto on_linear_velocity_limit_update(std::string source, messages::LinearVelocity::SharedPtr new_limit) -> void;

    rclcpp::TimerBase::SharedPtr m_limit_subscriber_timer{};

    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> m_limit_subscriptions{};

    std::optional<double> m_angular_velocity_limit{};
    std::optional<double> m_linear_velocity_limit{};
  };

}  // namespace boarai::intelligence

#endif