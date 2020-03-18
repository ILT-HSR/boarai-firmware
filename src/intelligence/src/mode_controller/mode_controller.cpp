#include "mode_controller/mode_controller.hpp"

#include "rclcpp/node.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"
#include "support/string_utility.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::intelligence
{
  auto constexpr node_name{"mode_controller"};

  mode_controller::mode_controller(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("mode_controller starting up");
    start_timers();
    start_services();
  }

  auto mode_controller::is_available(mode mode) -> bool
  {
    switch (mode)
    {
    case mode::none:
      return true;
    case mode::manual:
    case mode::autonomous:
      return m_angular_velocity_limit && m_linear_velocity_limit;
    default:
      return false;
    }
  }

  auto mode_controller::change_to(mode mode) -> void
  {
    if (mode == m_mode)
    {
      return;
    }

    switch ((m_mode = mode))
    {
    case mode::manual:
      m_gamepad_subscription = create_subscription<interface::topic::gamepad_input_t>(
          join("/", interface::ros_namespace, interface::topic::gamepad_input),
          1,
          std::bind(&mode_controller::on_gamepad_input_update, this, _1));
      break;
    default:
      m_gamepad_subscription.reset();
      break;
    }
  }

}  // namespace boarai::intelligence

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::intelligence::mode_controller)