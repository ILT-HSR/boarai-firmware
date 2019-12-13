#ifndef BOARAI_HARDWARE_MODBUSCPP_TCP_CONTEXT_HPP
#define BOARAI_HARDWARE_MODBUSCPP_TCP_CONTEXT_HPP

#include "modbuscpp/context.hpp"

#include <cstdint>
#include <string>
#include <system_error>
#include <variant>

namespace modbus
{
  auto constexpr default_modbus_tcp_port = std::uint16_t{502};

  struct tcp_context : context
  {
    explicit tcp_context(std::string slave_address, std::uint16_t port = default_modbus_tcp_port);

  private:
    explicit tcp_context(modbus_t * handle);
  };

}  // namespace modbus

#endif