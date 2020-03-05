#include "gps_provider/gps_provider.hpp"

#include "gps_provider/gpsmm_adapter.hpp"
#include "rcl_interfaces/msg/set_parameters_result.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/parameter.hpp"
#include "rclcpp/time_source.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "sensor_msgs/msg/nav_sat_status.hpp"
#include "std_msgs/msg/header.hpp"
#include "support/interfaces.hpp"

#include <cstdint>
#include <exception>
#include <mutex>
#include <string>
#include <vector>

using namespace std::literals;

auto constexpr node_name{"gps_provider"};

auto constexpr default_daemon_host{"localhost"};
auto constexpr default_daemon_port{static_cast<std::uint16_t>(2947)};

namespace boarai::hardware
{

  gps_provider::gps_provider(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
      , m_client{}
      , m_time_source_initialized{}
      , m_time_source{}
      , m_clock{std::make_shared<rclcpp::Clock>(RCL_ROS_TIME)}
      , m_position_publisher{}
      , m_position_update_mutex{}
  {
    auto guard = std::lock_guard{m_position_update_mutex};
    log_info("gps_provider starting up");
    declare_parameters();
    try
    {
      m_client.emplace(*this, daemon_host(), daemon_port());
      m_client->start();
    }
    catch (std::exception const &)
    {
      log_error("failed to connect to the gps daemon.");
    }

    m_position_publisher = create_publisher<topic::global_position_t>(topic::global_position, 10);
  }

  auto gps_provider::on_new_data(gps_data_t data) -> void
  {
    auto guard = std::lock_guard{m_position_update_mutex};

    if (!m_time_source_initialized)
    {
      m_time_source.attachNode(shared_from_this());
      m_time_source.attachClock(m_clock);
      m_time_source_initialized = true;
    }

    log_debug("Received new data: {} {} {}", data.fix.longitude, data.fix.latitude, data.fix.time.tv_sec);

    auto fix = sensor_msgs::msg::NavSatFix{};
    auto header = std_msgs::msg::Header{};
    auto status = sensor_msgs::msg::NavSatStatus{};

    header.frame_id = "global";
    header.stamp = m_clock->now();
    fix.header = header;

    status.service = sensor_msgs::msg::NavSatStatus::SERVICE_GPS;
    status.status =
        data.fix.mode < MODE_2D ? sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX : sensor_msgs::msg::NavSatStatus::STATUS_FIX;
    fix.status = status;

    if (fix.status.status != sensor_msgs::msg::NavSatStatus::STATUS_NO_FIX)
    {
      fix.latitude = data.fix.latitude;
      fix.longitude = data.fix.longitude;
      fix.altitude = data.fix.altitude;
      fix.position_covariance_type = sensor_msgs::msg::NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
      fix.position_covariance[0] = data.fix.epx;
      fix.position_covariance[4] = data.fix.epy;
      fix.position_covariance[8] = data.fix.epv;
    }

    m_position_publisher->publish(fix);
  }

  auto gps_provider::declare_parameters() -> void
  {
    declare_parameter(to_string(parameter::daemon_host), default_daemon_host);
    declare_parameter(to_string(parameter::daemon_port), default_daemon_port);
  }

  auto gps_provider::on_parameters_changed(std::vector<rclcpp::Parameter> new_parameters)
      -> rcl_interfaces::msg::SetParametersResult
  {
    auto result = rcl_interfaces::msg::SetParametersResult{};

    for (auto & param : new_parameters)
    {
      auto name = param.get_name();

      if (!is_valid<gps_provider::parameter>(name))
      {
        log_warning("received change of unknown parameter '{}'", name);
        result.successful = false;
        return result;
      }

      switch (from_string<gps_provider::parameter>(name))
      {
      case parameter::daemon_host:
        on_daemon_host_changed(param.as_string());
        break;
      case parameter::daemon_port:
        on_daemon_host_changed(param.as_string());
        break;
      case parameter::END_OF_ENUM:
        break;
      };
    }

    m_client.reset();
    try
    {
      m_client.emplace(*this, daemon_host(), daemon_port());
      m_client->start();
    }
    catch (std::exception const & e)
    {
      log_error("failed to connect to gps daemon");
    }

    return result;
  }

  auto gps_provider::on_daemon_port_changed(std::uint16_t new_value) -> void
  {
    log_info("chaning gps daemon port to {}", new_value);
  }

  auto gps_provider::on_daemon_host_changed(std::string new_value) -> void
  {
    log_info("chaning gps daemon host to {}", new_value);
  }

  auto gps_provider::daemon_host() -> std::string
  {
    auto result{""s};
    get_parameter_or(to_string(parameter::daemon_host), result, std::string{default_daemon_host});
    return result;
  }

  auto gps_provider::daemon_port() -> std::uint16_t
  {
    auto result = std::uint16_t{};
    get_parameter_or(to_string(parameter::daemon_port), result, default_daemon_port);
    return result;
  }

}  // namespace boarai::hardware

namespace boarai
{
  using namespace hardware;

  auto constexpr parameter_names = std::array{
      std::pair{gps_provider::parameter::daemon_host, "daemon_host"},
      std::pair{gps_provider::parameter::daemon_port, "daemon_port"},
  };

  static_assert(enum_mappings_are_unique(parameter_names), "missing mapping for parameter");
  static_assert(enum_map_has_all_entries(parameter_names, gps_provider::parameter::daemon_host),
                "duplicate key or value in parameter mappings");

  template<>
  auto to_string(gps_provider::parameter const & object) -> std::string
  {
    assert(is_valid<gps_provider::parameter>(static_cast<std::underlying_type_t<gps_provider::parameter>>(object)));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.first == object;
    });

    return found->second;
  }

  template<>
  auto from_string(std::string const & stringified) -> hardware::gps_provider::parameter
  {
    assert(is_valid<gps_provider::parameter>(stringified));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.second == stringified;
    });

    return found->first;
  }

  template<>
  auto is_valid<hardware::gps_provider::parameter>(std::underlying_type_t<hardware::gps_provider::parameter> candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }

  template<>
  auto is_valid<hardware::gps_provider::parameter>(std::string const & candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }
}  // namespace boarai

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::gps_provider)