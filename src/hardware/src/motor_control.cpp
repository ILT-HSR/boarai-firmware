#include "motor_control.hpp"

#include "rclcpp_components/register_node_macro.hpp"

#include <functional>

namespace boarai::hardware
{
  motor_control::motor_control(rclcpp::NodeOptions const & options)
      : Node{"motor_control", "hardware", options}
  {
    m_subscription =
        create_subscription<std_msgs::msg::Float32>("linear_velocity",
                                                    10,
                                                    std::bind(&motor_control::handle_message, this, std::placeholders::_1));
  }

  auto motor_control::handle_message(std_msgs::msg::Float32::SharedPtr message) -> void
  {
    RCLCPP_INFO(get_logger(), "Received: '%f'", message->data);
  }
}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::motor_control)
