#include "modbuscpp/client.hpp"

#include "modbuscpp/context.hpp"
#include "modbuscpp/error.hpp"

#include <errno.h>

#include <algorithm>
#include <iterator>
#include <type_traits>

namespace modbus
{
  client::client(connection const & connection)
      : m_context{connection.m_context}
  {
  }

  auto client::coil(address address) const noexcept -> sbit_writeable_datum
  {
    return {modbus_read_bits, modbus_write_bit, m_context, address, 1, modbus_error::too_many_coils_requested};
  }

  auto client::coils(address address, std::uint16_t count) const noexcept -> mbit_writeable_datum
  {
    return {modbus_read_bits, modbus_write_bits, m_context, address, count, modbus_error::too_many_coils_requested};
  }

  auto client::discrete_input(address address) const noexcept -> sbit_readable_datum
  {
    return {modbus_read_input_bits, m_context, address, 1, modbus_error::too_many_discrete_inputs_requested};
  }

  auto client::discrete_inputs(address address, std::uint16_t count) const noexcept -> mbit_readable_datum
  {
    return {modbus_read_input_bits, m_context, address, count, modbus_error::too_many_discrete_inputs_requested};
  }

  auto client::holding_register(address address) const noexcept -> sword_writeable_datum
  {
    return {modbus_read_input_registers,
            modbus_write_register,
            m_context,
            address,
            1,
            modbus_error::too_many_holding_registers_requested};
  }

  auto client::holding_registers(address address, std::uint16_t count) const noexcept -> mword_writeable_datum
  {
    return {modbus_read_input_registers,
            modbus_write_registers,
            m_context,
            address,
            count,
            modbus_error::too_many_holding_registers_requested};
  }

  auto client::input_register(address address) const noexcept -> sword_readable_datum
  {
    return {modbus_read_input_registers, m_context, address, 1, modbus_error::too_many_input_registers_requested};
  }

  auto client::input_registers(address address, std::uint16_t count) const noexcept -> mword_readable_datum
  {
    return {modbus_read_input_registers, m_context, address, count, modbus_error::too_many_input_registers_requested};
  }

}  // namespace modbus