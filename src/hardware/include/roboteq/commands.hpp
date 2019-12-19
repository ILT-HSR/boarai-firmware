#ifndef BOARAI_HARDWARE_ROBOTEQ_COMMANDS_HPP
#define BOARAI_HARDWARE_ROBOTEQ_COMMANDS_HPP

#include "roboteq/command.hpp"

namespace boarai::hardware::roboteq::commands
{
  auto constexpr emergency_stop = command{0x200c, command::argument_type::none};
  auto constexpr go_to_absolute_position = command{0x2001, command::argument_type::signed_32};
  auto constexpr go_to_relative_position = command{0x2010, command::argument_type::signed_32};
  auto constexpr set_motor_speed = command{0x2002, command::argument_type::signed_32};
  auto constexpr set_encoder_counter = command{0x2003, command::argument_type::signed_32};
  auto constexpr set_hall_counter = command{0x2004, command::argument_type::signed_32};
  auto constexpr set_motor_command = command{0x2000, command::argument_type::signed_32};
}  // namespace boarai::hardware::roboteq::commands

#endif