#include "velocity_estimator/velocity_estimator.hpp"

#include "estimation/layer_interface.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::estimation
{

  auto constexpr node_name{"velocity_estimator"};

  velocity_estimator::velocity_estimator(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("position_estimator starting up");

    start_subscriptions();
  }

  auto velocity_estimator::start_subscriptions() -> void
  {
    m_drive_velocity_subscription = create_subscription<hardware::topic::drive_velocity_t>(
        std::string{hardware::ros_namespace} + "/" + hardware::topic::drive_velocity,
        10,
        std::bind(&velocity_estimator::on_drive_velocity_update, this, _1));
  }

  auto velocity_estimator::on_drive_velocity_update(hardware::topic::drive_velocity_t::SharedPtr new_velocity) -> void
  {
    auto [linear_velocity, angular_velocity] = new_velocity->value;
    log_info("received drive velocity update: linear = {:.4f} m/s || angular = {:.4f} deg/s",
             linear_velocity,
             angular_velocity);
  }

}  // namespace boarai::estimation

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::estimation::velocity_estimator)