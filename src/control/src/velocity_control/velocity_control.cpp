#include "velocity_control/velocity_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"
#include "support/string_utility.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::control
{
  auto constexpr node_name{"velocity_control"};

  velocity_control::velocity_control(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    declare_parameters();
    m_controller_proportional.set_proportional_factor(proportional_factor());
    m_parameter_change_handler = add_on_set_parameters_callback(std::bind(&velocity_control::on_parameters_changed, this, _1));

    log_info("velocity_control starting up");

    m_drive_velocity_client = create_client<hardware::service::set_drive_velocity_t>(
        join("/", hardware::ros_namespace, hardware::service::set_drive_velocity));

    start_subscriptions();
    start_services();
    start_timers();
  }

}  // namespace boarai::control

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::control::velocity_control)