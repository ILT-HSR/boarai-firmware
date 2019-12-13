#ifndef BOARAI_HARDWARE_ROBOTEQ_CONSTANTS_HPP
#define BOARAI_HARDWARE_ROBOTEQ_CONSTANTS_HPP

#include <cstdint>

namespace boarai::hardware::roboteq
{
  enum struct commands : std::uint16_t
  {
    // TODO(smiracco): Add constants for commands
    do_some_stuff = 0xcafe,
  };

  enum struct queries : std::uint16_t
  {
    // TODO(smiracco): Add constants for queries
    read_some_stuff = 0xface,
  };

}  // namespace boarai::hardware::roboteq

#endif