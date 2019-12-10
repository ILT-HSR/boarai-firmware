#include "modbuscpp/tcp_context.hpp"

#include "modbuscpp/error.hpp"

#include <errno.h>

#include <modbus/modbus-tcp.h>

#include <system_error>

namespace modbus
{
  tcp_context::tcp_context(modbus_t * handle)
      : context{handle}
  {
  }

  auto tcp_context::create(std::string node_address, std::uint16_t node_port) -> std::variant<tcp_context, std::error_code>
  {
    auto api_context = modbus_new_tcp(node_address.c_str(), static_cast<int>(node_port));
    if (!api_context)
    {
      switch (errno)
      {
      case EINVAL:
        return make_error_code(modbus_error::invalid_ip_address);
      default:
        return make_error_code(static_cast<std::errc>(errno));
      }
    }
    return tcp_context{api_context};
  }
}  // namespace modbus
