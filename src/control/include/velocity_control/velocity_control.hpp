#ifndef BOARAI_CONTROL_VELOCITY_CONTROL_HPP
#define BOARAI_CONTROL_VELOCITY_CONTROL_HPP

#include "rclcpp/client.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"

namespace boarai::control
{
  struct velocity_control : fmt_node
  {
    using super = rclcpp::Node;

    explicit velocity_control(rclcpp::NodeOptions const & options);

  private:
    auto start_subscriptions() -> void;
    auto start_services() -> void;

    auto on_estimated_velocity_update(estimation::topic::estimated_velocity_t::SharedPtr new_velocity) -> void;
    auto on_set_target_velocity_request(service::set_target_velocity_t::Request::SharedPtr request,
                                        service::set_target_velocity_t::Response::SharedPtr) -> void;

    rclcpp::SubscriptionBase::SharedPtr m_estimated_velocity_subscription{};
    rclcpp::ServiceBase::SharedPtr m_set_target_velocity_service{};
    rclcpp::Client<hardware::service::set_drive_velocity_t>::SharedPtr m_drive_velocity_client{};
  };

}  // namespace boarai::control

#endif