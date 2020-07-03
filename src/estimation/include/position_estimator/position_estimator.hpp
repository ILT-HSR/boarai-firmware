#ifndef BOARAI_ESTIMATION_POSITION_ESTIMATOR_HPP
#define BOARAI_ESTIMATION_POSITION_ESTIMATOR_HPP

#include "rclcpp/node.hpp"
#include "rclcpp/service.hpp"
#include "rclcpp/subscription.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"

#include <optional>

namespace boarai::estimation
{

  struct position_estimator : fmt_node
  {
    using super = rclcpp::Node;

    explicit position_estimator(rclcpp::NodeOptions const & options);

  private:
    auto start_services();
    auto start_subscriptions();

    auto on_estimated_position_request(service::get_estimated_position_t::Request::SharedPtr request,
                                       service::get_estimated_position_t::Response::SharedPtr response) -> void;

    auto on_global_position_update(hardware::topic::global_position_t::SharedPtr new_position);
    auto on_imu_orientation_update(hardware::topic::imu_orientation_t::SharedPtr new_orientation);

    rclcpp::Service<service::get_estimated_position_t>::SharedPtr m_get_estimated_position_service{};

    rclcpp::Subscription<hardware::topic::global_position_t>::SharedPtr m_global_position_subscription{};
    rclcpp::Subscription<hardware::topic::imu_orientation_t>::SharedPtr m_imu_orientation_subscription{};

    std::optional<hardware::topic::global_position_t> m_last_global_position{};
    std::optional<hardware::topic::imu_orientation_t> m_last_imu_orientation{};
  };

}  // namespace boarai::estimation

#endif