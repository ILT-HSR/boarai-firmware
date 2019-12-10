#include "modbuscpp/connection.hpp"

#include <system_error>
#include <utility>

namespace modbus
{
  connection::connection(modbus::context context)
      : m_context{std::move(context)}
  {
    if (modbus_connect(m_context.handle().get()) < 0)
    {
      throw std::system_error{std::make_error_code(static_cast<std::errc>(errno))};
    }
  }

  connection::~connection()
  {
    close();
  }

  auto connection::close() noexcept -> void
  {
    modbus_close(m_context.handle().get());
  }

  auto connection::flush() noexcept -> std::error_code
  {
    if (modbus_flush(m_context.handle().get()))
    {
      return std::make_error_code(static_cast<std::errc>(errno));
    }
    return {};
  }

}  // namespace modbus