#ifndef BOARAI_SUPPORT_FMT_NODE_HPP
#define BOARAI_SUPPORT_FMT_NODE_HPP

#include "rclcpp/rclcpp.hpp"

#include <fmt/format.h>

#include <string>

namespace boarai
{

  struct fmt_node : rclcpp::Node
  {
  protected:
    explicit fmt_node(const std::string & node_name, const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    fmt_node(const std::string & node_name,
             const std::string & namespace_,
             const rclcpp::NodeOptions & options = rclcpp::NodeOptions{});

    template<typename... ParameterTypes>
    auto log_debug(std::string const & format, ParameterTypes &&... parameters)
    {
      auto message = fmt::format(format, std::forward<ParameterTypes>(parameters)...);
      RCLCPP_DEBUG(get_logger(), message);
    }

    template<typename... ParameterTypes>
    auto log_info(std::string const & format, ParameterTypes &&... parameters)
    {
      auto message = fmt::format(format, std::forward<ParameterTypes>(parameters)...);
      RCLCPP_INFO(get_logger(), message);
    }

    template<typename... ParameterTypes>
    auto log_warning(std::string const & format, ParameterTypes &&... parameters)
    {
      auto message = fmt::format(format, std::forward<ParameterTypes>(parameters)...);
      RCLCPP_WARN(get_logger(), message);
    }

    template<typename... ParameterTypes>
    auto log_error(std::string const & format, ParameterTypes &&... parameters)
    {
      auto message = fmt::format(format, std::forward<ParameterTypes>(parameters)...);
      RCLCPP_ERROR(get_logger(), message);
    }
  };

}  // namespace boarai

#endif