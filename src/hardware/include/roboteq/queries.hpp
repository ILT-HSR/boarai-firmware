#ifndef BOARAI_HARDWARE_ROBOTEQ_QUERIES_HPP
#define BOARAI_HARDWARE_ROBOTEQ_QUERIES_HPP

#include "roboteq/query.hpp"

#include <cstdint>

namespace boarai::hardware::roboteq::queries
{

  auto constexpr battery_voltage = query<std::uint16_t>{0x210d};
  auto constexpr brushless_motor_speed = query<std::int16_t>{0x210b};

}  // namespace boarai::hardware::roboteq::queries

#endif
