#include "modbuscpp/rtu_context.hpp"

#include "modbuscpp/error.hpp"

#include <errno.h>

#include <modbus/modbus-rtu.h>

#include <chrono>
#include <system_error>
namespace modbus
{
  namespace
  {
    template<typename ModeKind>
    auto set_mode(int (&setter)(modbus_t *, int), modbus_t * context, ModeKind mode, modbus_error invalid_error)
        -> std::error_code
    {
      if (setter(context, static_cast<int>(mode)) < 0)
      {
        switch (errno)
        {
        case EINVAL:
          return make_error_code(invalid_error);
        default:
          return make_error_code(static_cast<std::errc>(errno));
        }
      }
      return {};
    }

    template<typename ModeKind>
    auto get_mode(int (&getter)(modbus_t *), modbus_t * context) -> std::variant<ModeKind, std::error_code>
    {
      auto mode = getter(context);
      if (mode < 0)
      {
        return make_error_code(static_cast<std::errc>(errno));
      }
      return static_cast<ModeKind>(mode);
    }

    auto
    make_rtu_context(std::string node_device, std::uint32_t baud_rate, parity parity, data_bits data_bits, stop_bits stop_bits)
        -> modbus_t *
    {
      auto api_context = modbus_new_rtu(node_device.c_str(),
                                        static_cast<int>(baud_rate),
                                        static_cast<char>(parity),
                                        static_cast<int>(data_bits),
                                        static_cast<int>(stop_bits));

      if (!api_context)
      {
        throw std::system_error{make_error_code(static_cast<std::errc>(errno))};
      }

      return api_context;
    }
  }  // namespace

  rtu_context::rtu_context(std::string node_device,
                           std::uint32_t baud_rate,
                           parity parity,
                           data_bits data_bits,
                           stop_bits stop_bits)
      : context{make_rtu_context(node_device, baud_rate, parity, data_bits, stop_bits)}
  {
  }

  auto rtu_context::serial_mode(modbus::serial_mode mode) noexcept -> std::error_code
  {
    return set_mode(modbus_rtu_set_serial_mode, handle().get(), mode, modbus_error::invalid_serial_mode);
  }

  auto rtu_context::serial_mode() const noexcept -> std::variant<modbus::serial_mode, std::error_code>
  {
    return get_mode<modbus::serial_mode>(modbus_rtu_get_serial_mode, handle().get());
  }

  auto rtu_context::rts_mode(modbus::rts_mode mode) noexcept -> std::error_code
  {
    return set_mode(modbus_rtu_set_rts, handle().get(), mode, modbus_error::invalid_rts_mode);
  }

  auto rtu_context::rts_mode() const noexcept -> std::variant<modbus::rts_mode, std::error_code>
  {
    return get_mode<modbus::rts_mode>(modbus_rtu_get_rts, handle().get());
  }

  auto rtu_context::rts_delay(std::chrono::microseconds delay) noexcept -> std::error_code
  {
    if (modbus_rtu_set_rts_delay(handle().get(), static_cast<int>(delay.count())) < 0)
    {
      switch (errno)
      {
      case EINVAL:
        return make_error_code(modbus_error::invalid_rts_delay);
      default:
        return make_error_code(static_cast<std::errc>(errno));
      }
    }
    return {};
  }

  auto rtu_context::rts_delay() const noexcept -> std::variant<std::chrono::microseconds, std::error_code>
  {
    auto delay = modbus_rtu_get_rts_delay(handle().get());
    if (delay < 0)
    {
      return make_error_code(static_cast<std::errc>(errno));
    }
    return static_cast<std::chrono::microseconds>(delay);
  }

}  // namespace modbus