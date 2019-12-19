#ifndef BOARAI_HARDWARE_ROBOTEQ_CHANNEL_HPP
#define BOARAI_HARDWARE_ROBOTEQ_CHANNEL_HPP

#include <cstdint>

namespace boarai::hardware::roboteq
{
  enum struct channel : std::uint8_t
  {
    _1 = 1,
    velocity = 1,
    _2 = 2,
    steering = 2,
  };
}  // namespace boarai::hardware::roboteq

#endif