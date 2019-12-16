#ifndef BOARAI_HARDWARE_ROBOTEQ_CONSTANTS_HPP
#define BOARAI_HARDWARE_ROBOTEQ_CONSTANTS_HPP

#include <cstdint>

namespace boarai::hardware::roboteq
{
  enum struct commands : std::uint16_t
  {
    // TODO(smiracco): complete the list
    emergency_stop = 0x0180,
    go_to_absolute_position = 0x0020,
    go_to_relative_position = 0x01E0,
    set_motor_speed = 0x0040,
    set_encoder_counts = 0x0060,
    set_hall_counts = 0x0080

  };

  enum struct queries : std::uint16_t
  {
    // TODO(smiracco): Add constants for queries
    read_some_stuff = 0xface,
  };

}  // namespace boarai::hardware::roboteq

#endif
