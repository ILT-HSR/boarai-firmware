#ifndef BOARAI_HARDWARE_MODBUSCPP_RTU_CONTEXT_HPP
#define BOARAI_HARDWARE_MODBUSCPP_RTU_CONTEXT_HPP

#include "modbuscpp/context.hpp"

#include <modbus/modbus.h>

#include <chrono>
#include <cstddef>
#include <string>
#include <system_error>
#include <variant>

namespace modbus
{
  enum struct parity : char
  {
    none = 'N',
    even = 'E',
    odd = 'O',
  };

  enum struct data_bits : std::uint8_t
  {
    _5 = 5,
    _6 = 6,
    _7 = 7,
    _8 = 8,
  };

  enum struct stop_bits : std::uint8_t
  {
    _1 = 1,
    _2 = 2,
  };

  enum struct serial_mode : std::uint8_t
  {
    rs232 = MODBUS_RTU_RS232,
    rs485 = MODBUS_RTU_RS485,
  };

  enum struct rts_mode : std::uint8_t
  {
    none = MODBUS_RTU_RTS_NONE,
    up = MODBUS_RTU_RTS_UP,
    down = MODBUS_RTU_RTS_DOWN,
  };

  struct rtu_context : context
  {
    rtu_context(std::string node_device, std::uint32_t baud_rate, parity parity, data_bits data_bits, stop_bits stop_bits);

    auto serial_mode(serial_mode mode) noexcept -> std::error_code;

    auto serial_mode() const noexcept -> std::variant<modbus::serial_mode, std::error_code>;

    auto rts_mode(rts_mode mode) noexcept -> std::error_code;

    auto rts_mode() const noexcept -> std::variant<modbus::rts_mode, std::error_code>;

    auto rts_delay(std::chrono::microseconds delay) noexcept -> std::error_code;

    auto rts_delay() const noexcept -> std::variant<std::chrono::microseconds, std::error_code>;
  };
}  // namespace modbus
#endif