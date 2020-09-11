#ifndef BOARAI_CONTROL_VELOCITY_CONTROL_HPP
#define BOARAI_CONTROL_VELOCITY_CONTROL_HPP

#include "proportional_controller.hpp"
#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"
#include "support/string_utility.hpp"
#include "support/to_string.hpp"

namespace boarai::control
{
  struct velocity_control : fmt_node
  {
    enum struct parameter
    {
      proportional_factor,

      // End Marker for enum_utility support
      END_OF_ENUM
    };

    using super = rclcpp::Node;

    explicit velocity_control(rclcpp::NodeOptions const & options);

  private:
    auto declare_parameters() -> void;
    auto start_subscriptions() -> void;
    auto start_services() -> void;
    auto start_timers() -> void;

    auto proportional_factor() -> double;

    auto on_parameters_changed(std::vector<rclcpp::Parameter> new_parameters) -> rcl_interfaces::msg::SetParametersResult;
    auto on_proportional_factor_changed(double new_value) -> bool;

    auto on_estimated_velocity_update(estimation::topic::estimated_velocity_t::SharedPtr new_velocity) -> void;
    auto on_set_target_velocity_request(service::set_target_velocity_t::Request::SharedPtr request,
                                        service::set_target_velocity_t::Response::SharedPtr) -> void;
    auto on_velocity_update_timer_expired() -> void;

    rclcpp::TimerBase::SharedPtr m_velocity_update_timer{};

    rclcpp::SubscriptionBase::SharedPtr m_estimated_velocity_subscription{};
    rclcpp::ServiceBase::SharedPtr m_set_target_velocity_service{};
    rclcpp::Client<hardware::service::set_drive_velocity_t>::SharedPtr m_drive_velocity_client{};

    OnSetParametersCallbackHandle::SharedPtr m_parameter_change_handler{};

    std::optional<boarai_support::msg::PolarVelocity> m_velocity_target{};
    std::optional<boarai_support::msg::PolarVelocity> m_velocity_current{};

    proportional_controller<double> m_controller_proportional{};
  };

}  // namespace boarai::control

namespace boarai
{
  template<>
  auto to_string(control::velocity_control::parameter const & object) -> std::string;

  template<>
  auto from_string(std::string const & stringified) -> control::velocity_control::parameter;

  template<>
  auto is_valid<control::velocity_control::parameter>(std::underlying_type_t<control::velocity_control::parameter> candidate)
      -> bool;

  template<>
  auto is_valid<control::velocity_control::parameter>(std::string const & candidate) -> bool;

}  // namespace boarai

#endif