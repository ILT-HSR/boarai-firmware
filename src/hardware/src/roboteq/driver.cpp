#include "roboteq/driver.hpp"

#include "roboteq/channel.hpp"
#include "roboteq/commands.hpp"

#include <modbuscpp/client.hpp>

#include <system_error>

namespace boarai::hardware::roboteq
{
  driver::driver(modbus::client & client)
      : m_client{client}
  {
  }

  auto driver::emergency_stop() -> std::error_code
  {
    return commands::emergency_stop(m_client);
  }

  auto driver::go_to_position(channel channel, position position) -> std::error_code
  {
    return (position.type() == position_type::absolute
                ? commands::go_to_absolute_position
                : commands::go_to_relative_position)(m_client, position.encoder_count(), static_cast<std::uint16_t>(channel));
  }

  auto driver::set_motor_speed(channel channel, std::int32_t speed) -> std::error_code
  {
    return commands::set_motor_speed(m_client, speed, static_cast<std::uint16_t>(channel));
  }

  auto driver::set_encoder_counter(channel channel, std::int32_t counts) -> std::error_code
  {
    return commands::set_encoder_counter(m_client, counts, static_cast<std::uint16_t>(channel));
  }

  auto driver::set_hall_counter(channel channel, std::int32_t counts) -> std::error_code
  {
    return commands::set_hall_counter(m_client, counts, static_cast<std::uint16_t>(channel));
  }

}  // namespace boarai::hardware::roboteq
