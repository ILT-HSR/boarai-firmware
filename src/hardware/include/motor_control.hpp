#ifndef BOARAI_HARDWARE_MOTOR_CONTROL_HPP
#define BOARAI_HARDWARE_MOTOR_CONTROL_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"

#include <modbuscpp/client.hpp>
#include <modbuscpp/connection.hpp>
#include <modbuscpp/context.hpp>
#include <modbuscpp/tcp_context.hpp>

namespace boarai::hardware
{
  struct motor_control : rclcpp::Node
  {
    explicit motor_control(rclcpp::NodeOptions const & options);

  private:
    auto handle_message(std_msgs::msg::Float32::SharedPtr message) -> void;

    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr m_subscription;
    modbus::connection m_driver_connection;
    modbus::client m_driver_client{m_driver_connection};
  };
}  // namespace boarai::hardware

#endif