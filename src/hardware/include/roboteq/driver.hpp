#ifndef BOARAI_HARDWARE_ROBOTEQ_DRIVER_HPP
#define BOARAI_HARDWARE_ROBOTEQ_DRIVER_HPP

#include "modbuscpp/modbuscpp.hpp"
#include "roboteq/constants.hpp"

namespace boarai::hardware::roboteq
{
  struct driver
  {
    // TODO(smiracco): Implement useful driver API

    /**
     * Set the driver into emergency stop state
     * RoboteQ command EX
     */
    auto emergency_stop() -> std::error_code;

    /**
     * Go to absolute desired position
     * RoboteQ command P
     *
     * @param channel The target motor channel
     * @param count The absolute target position
     */
    auto go_to_absolute_position(channel channel, std::int32_t count) -> std::error_code;

    /**
     * Go to relative desired position
     * RoboteQ command PR
     *
     * @param channel The target motor channel
     * @param count A number of counts relative to the current position
     */
    auto go_to_relative_position(channel channel, std::int32_t delta) -> std::error_code;

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
    auto set_encoder_counts(channel channel, std::int32_t counts) -> std::error_code;

    /**
     * Set brushless counter (usually hall sensors)
     * RoboteQ command CB
     *
     * @param channel The target motor channel
     * @param counts The desired counts to set the hall sensor to
     */
    auto set_hall_counts(channel channel, std::int32_t counts) -> std::error_code;
  };
}  // namespace boarai::hardware::roboteq

#endif
