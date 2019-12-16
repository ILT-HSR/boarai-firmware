#ifndef BOARAI_HARDWARE_ROBOTEQ_COMMANDS_HPP
#define BOARAI_HARDWARE_ROBOTEQ_COMMANDS_HPP

#include <array>
#include <cstdint>
#include <utility>

namespace boarai::hardware::roboteq
{
  enum struct command_argument_type : std::uint8_t
  {
    none,
    boolean,
    signed_8,
    unsigned_8,
    signed_16,
    unsigned_16,
    signed_32,
    unsigned_32,
  };

  struct command
  {
    std::uint16_t id;
    command_argument_type argument_type;
  };

  namespace commands
  {
    auto constexpr emergency_stop = command{0x0180, command_argument_type::none};
    auto constexpr go_to_absolute_position = command{0x0020, command_argument_type::signed_32};
    auto constexpr go_to_relative_position = command{0x01E0, command_argument_type::signed_32};
    auto constexpr set_motor_speed = command{0x0040, command_argument_type::signed_32};
    auto constexpr set_encoder_counter = command{0x0060, command_argument_type::signed_32};
    auto constexpr set_hall_counter = command{0x0080, command_argument_type::signed_32};
  }  // namespace commands
}  // namespace boarai::hardware::roboteq

#endif
