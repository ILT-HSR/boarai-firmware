#ifndef BOARAI_HARDWARE_MODBUSCPP_CONTEXT_HPP
#define BOARAI_HARDWARE_MODBUSCPP_CONTEXT_HPP

#include <modbus/modbus.h>

#include <chrono>
#include <cstddef>
#include <memory>
#include <string>
#include <system_error>
#include <type_traits>

namespace modbus
{
  namespace detail
  {
    struct modbus_t_delete
    {
      auto operator()(modbus_t * handle) const noexcept -> void;
    };
  }  // namespace detail

  enum struct error_recovery_mode : std::underlying_type_t<modbus_error_recovery_mode>
  {
    none = MODBUS_ERROR_RECOVERY_NONE,
    link = MODBUS_ERROR_RECOVERY_LINK,
    protocol = MODBUS_ERROR_RECOVERY_PROTOCOL,
  };

  struct context
  {
    using handle_ptr = std::unique_ptr<modbus_t, detail::modbus_t_delete>;

    context(context const &) = delete;

    context(context &&) = default;

    ~context();

    auto operator=(context const &) -> context & = delete;

    auto operator=(context &&) -> context & = default;

    /// Slave configuration

    auto slave_id(int id) noexcept -> std::error_code;

    /// Debug mode

    auto debug(bool enable) noexcept -> void;

    /// Timeout settings

    auto byte_timeout() const noexcept -> std::chrono::microseconds;

    auto byte_timeout(std::chrono::seconds duration) noexcept -> std::error_code;

    auto byte_timeout(std::chrono::milliseconds duration) noexcept -> std::error_code;

    auto byte_timeout(std::chrono::microseconds duration) noexcept -> std::error_code;

    auto indication_timeout() const noexcept -> std::chrono::microseconds;

    auto indication_timeout(std::chrono::seconds duration) noexcept -> std::error_code;

    auto indication_timeout(std::chrono::milliseconds duration) noexcept -> std::error_code;

    auto indication_timeout(std::chrono::microseconds duration) noexcept -> std::error_code;

    auto response_timeout() const noexcept -> std::chrono::microseconds;

    auto response_timeout(std::chrono::seconds duration) noexcept -> std::error_code;

    auto response_timeout(std::chrono::milliseconds duration) noexcept -> std::error_code;

    auto response_timeout(std::chrono::microseconds duration) noexcept -> std::error_code;

    /// C API Access

    auto handle() const -> handle_ptr const &;

  protected:
    explicit context(modbus_t * handle) noexcept;

  private:
    handle_ptr m_handle;
  };
}  // namespace modbus

#endif