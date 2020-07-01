#include "velocity_control/velocity_control.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "rmw/qos_profiles.h"
#include "support/interfaces.hpp"
#include "support/string_utility.hpp"

#include <functional>

using namespace std::placeholders;

namespace boarai::control
{
  auto constexpr node_name{"velocity_control"};

  // static const rmw_qos_profile_t rmw_qos_profile_services_default =
  // {
  //   RMW_QOS_POLICY_HISTORY_KEEP_LAST,
  //   10,
  //   RMW_QOS_POLICY_RELIABILITY_RELIABLE,
  //   RMW_QOS_POLICY_DURABILITY_VOLATILE,
  //   RMW_QOS_DEADLINE_DEFAULT,
  //   RMW_QOS_LIFESPAN_DEFAULT,
  //   RMW_QOS_POLICY_LIVELINESS_SYSTEM_DEFAULT,
  //   RMW_QOS_LIVELINESS_LEASE_DURATION_DEFAULT,
  //   false
  // };

  velocity_control::velocity_control(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    log_info("velocity_control starting up");

    auto drive_velocity_policy = rmw_qos_profile_services_default;
    drive_velocity_policy.depth = 1;
    m_drive_velocity_client = create_client<hardware::service::set_drive_velocity_t>(
        join("/", hardware::ros_namespace, hardware::service::set_drive_velocity),
        drive_velocity_policy);

    start_subscriptions();
    start_services();
  }

  auto velocity_control::start_services() -> void
  {
    m_set_target_velocity_service = create_service<service::set_target_velocity_t>(
        service::set_target_velocity,
        std::bind(&velocity_control::on_set_target_velocity_request, this, _1, _2));
  }

  auto velocity_control::on_set_target_velocity_request(service::set_target_velocity_t::Request::SharedPtr request,
                                                        service::set_target_velocity_t::Response::SharedPtr) -> void
  {
    auto velocity = request->velocity.value;
    log_info("received request to set velocity to: linear: {} || angular: {}", velocity.r, velocity.phi);

    if (m_drive_velocity_client->service_is_ready())
    {
      log_debug("calling '{}' with linear=={} and angular=={}",
                m_drive_velocity_client->get_service_name(),
                velocity.r,
                velocity.phi);
      auto hw_request = std::make_shared<hardware::service::set_drive_velocity_t::Request>();
      hw_request->velocity = messages::PolarVelocity{}.set__value(velocity);
      m_drive_velocity_client->async_send_request(hw_request);
    }
  }

}  // namespace boarai::control

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::control::velocity_control)