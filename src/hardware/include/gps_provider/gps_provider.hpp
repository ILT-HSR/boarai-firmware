#ifndef BOARAI_HARDWARE_GPS_PROVIDER_HPP
#define BOARAI_HARDWARE_GPS_PROVIDER_HPP

#include "gps_provider/gpsd_client.hpp"
#include "gps_provider/gpsmm_adapter.hpp"
#include "hardware/layer_interface.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time_source.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "support/enum_utility.hpp"
#include "support/fmt_node.hpp"
#include "support/to_string.hpp"

#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace boarai::hardware
{

  struct gps_provider
      : fmt_node
      , gpsd_client::listener
  {
    using super = rclcpp::Node;

    using super::declare_parameters;

    enum struct parameter
    {
      daemon_host,
      daemon_port,

      // End Marker for enum_utility support
      END_OF_ENUM
    };

    explicit gps_provider(rclcpp::NodeOptions const & options);

    auto on_new_data(gps_data_t data) -> void override;

  private:
    auto declare_parameters() -> void;

    auto on_parameters_changed(std::vector<rclcpp::Parameter> new_parameters) -> rcl_interfaces::msg::SetParametersResult;
    auto on_daemon_port_changed(std::uint16_t new_value) -> void;
    auto on_daemon_host_changed(std::string new_value) -> void;

    auto daemon_host() -> std::string;
    auto daemon_port() -> std::uint16_t;

    std::optional<gpsd_client> m_client;
    bool m_time_source_initialized;
    rclcpp::TimeSource m_time_source;
    rclcpp::Clock::SharedPtr m_clock;

    rclcpp::Publisher<topic::global_position_t>::SharedPtr m_position_publisher;
    std::mutex m_position_update_mutex;
  };

}  // namespace boarai::hardware

namespace boarai
{

  template<>
  auto to_string(hardware::gps_provider::parameter const & object) -> std::string;

  template<>
  auto from_string(std::string const & stringified) -> hardware::gps_provider::parameter;

  template<>
  auto is_valid<hardware::gps_provider::parameter>(std::underlying_type_t<hardware::gps_provider::parameter> candidate) -> bool;

  template<>
  auto is_valid<hardware::gps_provider::parameter>(std::string const & candidate) -> bool;

}  // namespace boarai

#endif