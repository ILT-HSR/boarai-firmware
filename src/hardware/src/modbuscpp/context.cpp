#include "modbuscpp/context.hpp"

#include "modbuscpp/error.hpp"

#include <errno.h>

#include <chrono>
#include <system_error>

namespace modbus
{
  namespace detail
  {
    auto modbus_t_delete::operator()(modbus_t * handle) const noexcept -> void
    {
      modbus_free(handle);
    }

    auto set_timeout(int (&setter)(modbus_t *, std::uint32_t, std::uint32_t),
                     context::handle_ptr const & handle,
                     std::chrono::microseconds duration) -> std::error_code
    {
      auto duration_as_seconds = std::chrono::duration_cast<std::chrono::seconds>(duration);
      auto seconds = static_cast<std::uint32_t>(duration_as_seconds.count());
      auto microseconds = static_cast<std::uint32_t>((duration - duration_as_seconds).count());

      if (setter(handle.get(), seconds, microseconds) < 0)
      {
        return make_error_code(modbus_error::invalid_duration);
      }
      return {};
    }

    auto get_timeout(int (&getter)(modbus_t *, std::uint32_t *, std::uint32_t *), context::handle_ptr const & handle)
        -> std::chrono::microseconds
    {
      auto seconds = std::uint32_t{};
      auto microseconds = std::uint32_t{};
      getter(handle.get(), &seconds, &microseconds);
      return std::chrono::seconds{seconds} + std::chrono::microseconds{microseconds};
    }
  }  // namespace detail

  context::~context() = default;

  context::context(modbus_t * handle) noexcept
      : m_handle{handle}
  {
  }

  /// Timeout settings

  auto context::byte_timeout() const noexcept -> std::chrono::microseconds
  {
    return get_timeout(modbus_get_byte_timeout, m_handle);
  }

  auto context::byte_timeout(std::chrono::seconds duration) noexcept -> std::error_code
  {
    return byte_timeout(std::chrono::duration_cast<std::chrono::microseconds>(duration));
  }

  auto context::byte_timeout(std::chrono::milliseconds duration) noexcept -> std::error_code
  {
    return byte_timeout(std::chrono::duration_cast<std::chrono::microseconds>(duration));
  }

  auto context::byte_timeout(std::chrono::microseconds duration) noexcept -> std::error_code
  {
    return set_timeout(modbus_set_byte_timeout, m_handle, duration);
  }

  auto context::indication_timeout() const noexcept -> std::chrono::microseconds
  {
    return get_timeout(modbus_get_indication_timeout, m_handle);
  }

  auto context::indication_timeout(std::chrono::seconds duration) noexcept -> std::error_code
  {
    return indication_timeout(std::chrono::duration_cast<std::chrono::microseconds>(duration));
  }

  auto context::indication_timeout(std::chrono::milliseconds duration) noexcept -> std::error_code
  {
    return indication_timeout(std::chrono::duration_cast<std::chrono::microseconds>(duration));
  }

  auto context::indication_timeout(std::chrono::microseconds duration) noexcept -> std::error_code
  {
    return set_timeout(modbus_set_indication_timeout, m_handle, duration);
  }

  auto context::response_timeout() const noexcept -> std::chrono::microseconds
  {
    return get_timeout(modbus_get_response_timeout, m_handle);
  }

  auto context::response_timeout(std::chrono::seconds duration) noexcept -> std::error_code
  {
    return response_timeout(std::chrono::duration_cast<std::chrono::microseconds>(duration));
  }

  auto context::response_timeout(std::chrono::milliseconds duration) noexcept -> std::error_code
  {
    return response_timeout(std::chrono::duration_cast<std::chrono::microseconds>(duration));
  }

  auto context::response_timeout(std::chrono::microseconds duration) noexcept -> std::error_code
  {
    return set_timeout(modbus_set_response_timeout, m_handle, duration);
  }

  /// Configuration management

  auto context::debug(bool enable) noexcept -> void
  {
    modbus_set_debug(m_handle.get(), enable);
  }

  auto context::slave_id(int id) noexcept -> std::error_code
  {
    if (modbus_set_slave(m_handle.get(), id) < 0)
    {
      return make_error_code(modbus_error::invalid_slave_number);
    }
    return {};
  }

  /// C API Access

  auto context::handle() const -> handle_ptr const &
  {
    return m_handle;
  }

}  // namespace modbus