#ifndef BOARAI_HARDWARE_MODBUSCPP_TCP_CONTEXT_HPP
#define BOARAI_HARDWARE_MODBUSCPP_TCP_CONTEXT_HPP

#include "modbuscpp/context.hpp"

#include <cstdint>
#include <string>
#include <system_error>
#include <variant>

namespace modbus
{
  struct tcp_context : context
  {
    auto static create(std::string node_address, std::uint16_t node_port) -> std::variant<tcp_context, std::error_code>;

  private:
    explicit tcp_context(modbus_t * handle);
  };

}  // namespace modbus

#endif