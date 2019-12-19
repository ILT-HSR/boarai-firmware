#ifndef BOARAI_HARDWARE_ROBOTEQ_QUERIES_HPP
#define BOARAI_HARDWARE_ROBOTEQ_QUERIES_HPP

#include <cstdint>

namespace boarai::hardware::roboteq
{
  enum struct query : std::uint16_t
  {
    // TODO(smiracco): Add constants for queries
    read_some_stuff = 0xface,
  };

}  // namespace boarai::hardware::roboteq

#endif
