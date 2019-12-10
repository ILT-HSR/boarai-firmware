#include "modbuscpp/error.hpp"

namespace modbus
{
  char const * modbus_category_impl::name() const noexcept
  {
    return "MODBUS errors";
  }

  std::string modbus_category_impl::message(int code) const
  {
    switch (static_cast<modbus_error>(code))
    {
    case modbus_error::invalid_slave_number:
      return "Invalid MODBUS slave number";
    case modbus_error::invalid_context:
      return "Invalid MODBUS context";
    case modbus_error::invalid_duration:
      return "Invalid duration";
    case modbus_error::invalid_ip_address:
      return "Invalid IP address";
    case modbus_error::invalid_serial_mode:
      return "Invalid serial mode";
    case modbus_error::invalid_rts_mode:
      return "Invalid RTS mode";
    case modbus_error::invalid_rts_delay:
      return "Invalid RTS delay";
    case modbus_error::too_many_coils_requested:
      return "Too many coils requested";
    case modbus_error::too_many_discrete_inputs_requested:
      return "Too many discrete inputs requested";
    case modbus_error::too_many_holding_registers_requested:
      return "Too many holding registers requested";
    case modbus_error::too_many_input_registers_requested:
      return "Too many input registers requested";
    default:
      return "unknown error";
    }
  }

  std::error_condition modbus_category_impl::default_error_condition(int code) const noexcept
  {
    switch (static_cast<modbus_error>(code))
    {
    case modbus_error::invalid_context:
    case modbus_error::invalid_slave_number:
    case modbus_error::invalid_duration:
    case modbus_error::invalid_ip_address:
    case modbus_error::invalid_serial_mode:
    case modbus_error::invalid_rts_mode:
    case modbus_error::invalid_rts_delay:
    case modbus_error::too_many_coils_requested:
    case modbus_error::too_many_discrete_inputs_requested:
    case modbus_error::too_many_holding_registers_requested:
    case modbus_error::too_many_input_registers_requested:
      return make_error_condition(std::errc::invalid_argument);
    default:
      return std::error_condition(code, *this);
    }
  }

  std::error_category const & modbus_category()
  {
    static modbus_category_impl const category{};
    return category;
  }

  std::error_code make_error_code(modbus_error result)
  {
    return {static_cast<int>(result), modbus_category()};
  }

  std::error_condition make_error_condition(modbus_error result)
  {
    return {static_cast<int>(result), modbus_category()};
  }

}  // namespace modbus
