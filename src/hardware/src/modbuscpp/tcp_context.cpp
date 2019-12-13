#include "modbuscpp/tcp_context.hpp"

#include "modbuscpp/error.hpp"

#include <errno.h>

#include <modbus/modbus-tcp.h>

#include <cstdint>
#include <system_error>

namespace modbus
{
  namespace
  {
    auto make_tcp_context(std::string address, std::uint16_t port) -> modbus_t *
    {
      auto api_context = modbus_new_tcp(address.c_str(), static_cast<int>(port));
      if (!api_context)
      {
        switch (errno)
        {
        case EINVAL:
          throw std::system_error{make_error_code(modbus_error::invalid_ip_address)};
        default:
          throw std::system_error{make_error_code(static_cast<std::errc>(errno))};
        }
      }
      return api_context;
    }
  }  // namespace

  tcp_context::tcp_context(std::string slave_address, std::uint16_t port)
      : context{make_tcp_context(slave_address, port)}
  {
  }

}  // namespace modbus
