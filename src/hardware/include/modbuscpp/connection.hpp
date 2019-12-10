#ifndef BOARAI_HARDWARE_MODBUSCPP_CONNECTION_HPP
#define BOARAI_HARDWARE_MODBUSCPP_CONNECTION_HPP

#include <modbuscpp/context.hpp>

namespace modbus
{
  struct connection
  {
    explicit connection(context context);

    ~connection() noexcept;

    auto close() noexcept -> void;

    auto flush() noexcept -> std::error_code;

  private:
    friend struct client;

    modbus::context m_context;
  };
}  // namespace modbus

#endif