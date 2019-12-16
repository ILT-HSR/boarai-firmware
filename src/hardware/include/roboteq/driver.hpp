#ifndef BOARAI_HARDWARE_ROBOTEQ_DRIVER_HPP
#define BOARAI_HARDWARE_ROBOTEQ_DRIVER_HPP

#include "modbuscpp/client.hpp"
#include "roboteq/channel.hpp"
#include "roboteq/position.hpp"

#include <system_error>

namespace boarai::hardware::roboteq
{
  struct driver
  {
    // TODO(smiracco): Implement useful driver API

    driver(modbus::client & client);

    /**
     * Set the driver into emergency stop state
     * RoboteQ command EX
     */
    auto emergency_stop() -> std::error_code;

    /**
     * Go to the specified position
     *
     * @note This function is equivalent to the Roboteq command P or PR depending on the position type
     *
     * @param channel The target motor channel
     * @param position The desired motoer position
     */
    auto go_to_position(channel channel, position position) -> std::error_code;

    /**
     * Set motor speed
     * RoboteQ command S
     *
     * @param channel The target motor channel
     * @param speed The desired Speed in rpm
     */
    auto set_motor_speed(channel channel, std::int32_t speed) -> std::error_code;

    /**
     * Set encoder counter
     * RoboteQ command C
     *
     * @param channel The target motor channel
     * @param counts The desired counts to set the encoder to
     */
    auto set_encoder_counter(channel channel, std::int32_t counts) -> std::error_code;

    /**
     * Set brushless counter (usually hall sensors)
     * RoboteQ command CB
     *
     * @param channel The target motor channel
     * @param counts The desired counts to set the hall sensor to
     */
    auto set_hall_counter(channel channel, std::int32_t counts) -> std::error_code;

  private:
    modbus::client & m_client;
  };

}  // namespace boarai::hardware::roboteq

#endif