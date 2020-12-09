#ifndef BOARAI_HARDWARE_DEBUG_TANK_DRIVE_HPP
#define BOARAI_HARDWARE_DEBUG_TANK_DRIVE_HPP

#include "rclcpp/node_options.hpp"
#include "rclcpp/publisher.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/interfaces.hpp"
#include "support/messages.hpp"
#include "support/to_string.hpp"

#include <functional>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <type_traits>

namespace boarai::hardware
{
  struct tank_drive_debug : fmt_node
  {

    enum struct parameter
    {
      wheel_spacing,
      maximum_linear_velocity,
      acceleration_delay,

      // End Marker for enum_utility support
      END_OF_ENUM
    };

    explicit tank_drive_debug(rclcpp::NodeOptions const & options);

  private:
    auto declare_parameters() -> void;
    auto start_services() -> void;
    auto start_timers() -> void;
    auto start_publishers() -> void;
    auto publish_limits() -> void;

    auto wheel_spacing() -> double;
    auto maximum_linear_velocity() -> double;

    auto maximum_angular_velocity() -> double;

    auto acceleration_delay() -> double;

    auto on_drive_velocity_request(service::set_drive_velocity_t::Request::SharedPtr request,
                                   service::set_drive_velocity_t::Response::SharedPtr response) -> void;
    auto on_angular_velocity_request(service::get_maximum_angular_velocity_t::Request::SharedPtr request,
                                     service::get_maximum_angular_velocity_t::Response::SharedPtr response) -> void;

    auto on_drive_velocity_update_timer_expired() -> void;

    rclcpp::Service<service::set_drive_velocity_t>::SharedPtr m_drive_velocity_service{};
    rclcpp::Service<service::get_maximum_angular_velocity_t>::SharedPtr m_angular_velocity_service{};

    rclcpp::TimerBase::SharedPtr m_drive_velocity_update_timer{};

    rclcpp::Publisher<topic::drive_velocity_t>::SharedPtr m_drive_velocity_publisher{};

    rclcpp::Publisher<limit::angular_velocity_t>::SharedPtr m_angular_velocity_limit_publisher{};
    rclcpp::Publisher<limit::linear_velocity_t>::SharedPtr m_linear_velocity_limit_publisher{};

    std::optional<boarai::messages::PolarVelocity> m_requested_velocity{};
    boarai::messages::PolarVelocity m_current_velocity{};
    std::mutex m_command_mutex{};
  };
}  // namespace boarai::hardware

namespace boarai
{
  template<>
  auto to_string(hardware::tank_drive_debug::parameter const & object) -> std::string;

  template<>
  auto from_string(std::string const & stringified) -> hardware::tank_drive_debug::parameter;

  template<>
  auto is_valid<hardware::tank_drive_debug::parameter>(std::underlying_type_t<hardware::tank_drive_debug::parameter> candidate)
      -> bool;

  template<>
  auto is_valid<hardware::tank_drive_debug::parameter>(std::string const & candidate) -> bool;

}  // namespace boarai

#endif