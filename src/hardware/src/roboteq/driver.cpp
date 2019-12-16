#include "roboteq/driver.hpp"

#include "modbuscpp/modbuscpp.hpp"
#include "roboteq/constants.hpp"

#include <system_error>

namespace boarai::hardware::roboteq
{
  namespace helpers
  {
  }
  driver::driver()
  {
    // TODO (smiracco): define proper constructor
    auto context{modbus::context(modbus::tcp_context("192.168.1.20"))};
    auto connection{modbus::connection(context)};
    auto m_client{modbus::client(connection)};
  }

  auto driver::command(commands command, channel channel, std::int32_t value) -> std::error_code
  {
    unsigned int value_LE = value;
    return m_client.holding_registers(command + channel, 2) = {0x0000, 0x0000l};
  }

  auto driver::emergency_stop() -> std::error_code
  {
    return m_client.holding_registers(emergency_stop, 2) = {0x0000, 0x0000};
  }

}  // namespace boarai::hardware::roboteq
