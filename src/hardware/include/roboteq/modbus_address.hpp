#ifndef BOARAI_HARDWARE_ROBOTEQ_MODBUS_ADDRESS_HPP
#define BOARAI_HARDWARE_ROBOTEQ_MODBUS_ADDRESS_HPP

#include "modbuscpp/address.hpp"

#include <cstdint>

namespace boarai::hardware::roboteq
{

  /**
   * @internal
   *
   * Calculate the MODBUS address of the command for a given index
   *
   * @param index The index of the command (e.g. a motor channel)
   * @return The indexed MODBUS address of the command
   */
  auto constexpr modbus_address(std::uint16_t can_id, std::uint16_t index) noexcept -> modbus::address
  {
    auto base = static_cast<std::uint32_t>(can_id) << 5;
    auto indexed = base | index;
    return modbus::address{static_cast<std::uint16_t>(indexed & 0xffff)};
  }

}  // namespace boarai::hardware::roboteq

#endif