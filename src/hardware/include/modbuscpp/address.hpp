#ifndef BOARAI_HARDWARE_MODBUSCPP_ADDRESS_HPP
#define BOARAI_HARDWARE_MODBUSCPP_ADDRESS_HPP

#include <cstdint>
#include <stdexcept>
#include <string>

namespace modbus
{
  auto constexpr address_upper_bound = 0xffff;

  struct address
  {
    explicit constexpr address(std::uint16_t value)
        : m_value{value}
    {
    }

    explicit constexpr operator int() const
    {
      return m_value;
    }

  private:
    std::uint16_t m_value;
  };

  inline namespace modbus_literals
  {
    auto constexpr operator""_addr(unsigned long long const value) -> address
    {
      if (value > address_upper_bound)
      {
        throw std::invalid_argument{"Address " + std::to_string(value) + " is outside of legal range [0, 65535]"};
      }
      return address{static_cast<std::uint16_t>(value)};
    }
  }  // namespace modbus_literals
}  // namespace modbus

#endif