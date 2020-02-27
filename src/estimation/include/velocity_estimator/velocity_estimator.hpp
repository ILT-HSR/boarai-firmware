#ifndef BOARAI_ESTIMATION_POSITION_ESTIMATOR_HPP
#define BOARAI_ESTIMATION_POSITION_ESTIMATOR_HPP

#include "hardware/layer_interface.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/node_options.hpp"
#include "rclcpp/subscription.hpp"
#include "support/fmt_node.hpp"

namespace boarai::estimation
{

  struct velocity_estimator : fmt_node
  {
    using super = rclcpp::Node;

    explicit velocity_estimator(rclcpp::NodeOptions const & options);

  private:
    auto start_subscriptions() -> void;

    auto on_drive_velocity_update(hardware::topic::drive_velocity_t::SharedPtr new_velocity) -> void;

    rclcpp::Subscription<hardware::topic::drive_velocity_t>::SharedPtr m_drive_velocity_subscription{};
  };

}  // namespace boarai::estimation

#endif