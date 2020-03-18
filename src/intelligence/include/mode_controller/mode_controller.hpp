#ifndef BOARAI_INTELLIGENCE_MODE_CONTROLLER_HPP
#define BOARAI_INTELLIGENCE_MODE_CONTROLLER_HPP

#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp/timer.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"
#include "support/messages.hpp"
#include "support/to_string.hpp"

#include <map>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace boarai::intelligence
{

  struct mode_controller : fmt_node
  {
    enum struct mode
    {
      none,
      manual,
      autonomous,

      // End Marker for enum_utility support
      END_OF_ENUM
    };

    using super = rclcpp::Node;

    explicit mode_controller(rclcpp::NodeOptions const & options);

  private:
    auto start_timers() -> void;
    auto start_services() -> void;

    auto is_available(mode mode) -> bool;
    auto change_to(mode mode) -> void;

    auto subscribe_to_limit_topic(std::string name, std::string type) -> void;

    auto on_limit_subscriber_timer_expired() -> void;

    auto on_agular_velocity_limit_update(std::string source, messages::AngularVelocity::SharedPtr new_limit) -> void;
    auto on_linear_velocity_limit_update(std::string source, messages::LinearVelocity::SharedPtr new_limit) -> void;
    auto on_gamepad_input_update(interface::topic::gamepad_input_t::SharedPtr new_input) -> void;
    auto on_set_mode_request(service::set_mode_t::Request::SharedPtr request, service::set_mode_t::Response::SharedPtr response)
        -> void;

    rclcpp::TimerBase::SharedPtr m_limit_subscriber_timer{};

    rclcpp::ServiceBase::SharedPtr m_set_mode_service{};

    rclcpp::SubscriptionBase::SharedPtr m_gamepad_subscription{};

    std::map<std::string, rclcpp::SubscriptionBase::SharedPtr> m_limit_subscriptions{};

    std::optional<double> m_angular_velocity_limit{};
    std::optional<double> m_linear_velocity_limit{};
    mode m_mode{mode::none};
  };

}  // namespace boarai::intelligence

namespace boarai
{

  template<>
  auto to_string(intelligence::mode_controller::mode const & object) -> std::string;

  template<>
  auto from_string(std::string const & stringified) -> intelligence::mode_controller::mode;

  template<>
  auto is_valid<intelligence::mode_controller::mode>(std::underlying_type_t<intelligence::mode_controller::mode> candidate)
      -> bool;

  template<>
  auto is_valid<intelligence::mode_controller::mode>(std::string const & candidate) -> bool;

}  // namespace boarai

#endif