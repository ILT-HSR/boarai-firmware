#include "modbuscpp/test_server.hpp"

#include "modbuscpp/address.hpp"
#include "modbuscpp/error.hpp"
#include "modbuscpp/tcp_context.hpp"

#include <errno.h>
#include <unistd.h>

#include <modbus/modbus.h>

#include <array>
#include <mutex>
#include <system_error>

namespace boarai::hardware::test
{
  auto detail::modbus_mapping_t_delete::operator()(modbus_mapping_t * mapping) const noexcept -> void
  {
    modbus_mapping_free(mapping);
  }

  test_server::test_server(mapping_parameters mapping_parameters, std::uint16_t port)
      : m_context{*modbus::tcp_context::create("127.0.0.1", port)}
  {
    auto [coils, discrete_inputs, holding_registers, input_registers] = mapping_parameters;
    auto data_guard = std::lock_guard{m_data_mutex};
    auto api_mapping = modbus_mapping_new(coils, discrete_inputs, holding_registers, input_registers);
    if (!api_mapping)
    {
      throw std::system_error{make_error_code(static_cast<std::errc>(errno))};
    }
    m_data = mapping_pointer{api_mapping};
    m_socket = modbus_tcp_listen(m_context.handle().get(), 1);
  }

  test_server::~test_server()
  {
    close(m_socket);
  }

  auto test_server::run_once() -> void
  {
    auto data_guard = std::lock_guard{m_data_mutex};
    modbus_tcp_accept(m_context.handle().get(), &m_socket);

    auto query = std::array<std::byte, MODBUS_TCP_MAX_ADU_LENGTH>{};
    auto received = modbus_receive(m_context.handle().get(), reinterpret_cast<std::uint8_t *>(query.data()));
    if (received > 0)
    {
      modbus_reply(m_context.handle().get(), reinterpret_cast<std::uint8_t *>(query.data()), received, m_data.get());
    }
    else if (received < 0 && errno != ECONNRESET)
    {
      throw std::system_error{make_error_code(static_cast<std::errc>(errno))};
    }
  }

  auto test_server::coil(modbus::address address) const -> bool
  {
    auto data_guard = std::lock_guard{m_data_mutex};
    return m_data->tab_bits[static_cast<int>(address)];
  }

  auto test_server::coil(modbus::address address, bool value) -> void
  {
    auto data_guard = std::lock_guard{m_data_mutex};
    m_data->tab_bits[static_cast<int>(address)] = value;
  }

}  // namespace boarai::hardware::test