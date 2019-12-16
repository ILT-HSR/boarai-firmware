#ifndef BOARAI_HARDWARE_ROBOTEQ_CHANNEL_HPP
#define BOARAI_HARDWARE_ROBOTEQ_CHANNEL_HPP

#include <cstdint>

namespace boarai::hardware::roboteq
{
  enum struct channel : std::uint8_t
  {

    channel_01 = 1,
    channel_02 = 2,
  };

}  // namespace boarai::hardware::roboteq

#endif