#include "roboteq/command.hpp"

#include "roboteq/modbus_address.hpp"

#include <string>
#include <system_error>

namespace boarai::hardware::roboteq
{
  char const * command_category_impl::name() const noexcept
  {
    return "Roboteq command error";
  }

  std::string command_category_impl::message(int error) const
  {
    using namespace std::string_literals;

    switch (static_cast<command_error>(error))
    {
    case command_error::missing_argument:
      return "Missing argument for command"s;
    case command_error::unexpected_argument:
      return "Unexpected argument for command that does not take any arguments"s;
    case command_error::argument_type_mismatch:
      return "Supplied argument type does not match the expected type"s;
    }

    return "Unknown command error"s;
  }

  std::error_condition command_category_impl::default_error_condition(int error) const noexcept
  {
    switch (static_cast<command_error>(error))
    {
    case command_error::missing_argument:
    case command_error::unexpected_argument:
    case command_error::argument_type_mismatch:
      return std::errc::invalid_argument;
    default:
      return std::error_condition(error, *this);
    }
  }

  auto command_category() -> std::error_category const &
  {
    static command_category_impl const instance;
    return instance;
  }

  auto make_error_code(command_error code) -> std::error_code
  {
    return {static_cast<int>(code), command_category()};
  }

  auto make_error_condition(command_error code) -> std::error_condition
  {
    return {static_cast<int>(code), command_category()};
  }

  auto command::operator()(modbus::client & client, std::uint16_t index) const -> std::error_code
  {
    return do_execute(client, {0, 0}, index);
  }

  auto command::do_execute(modbus::client & client, std::pair<std::uint16_t, std::uint16_t> argument, std::uint16_t index) const
      -> std::error_code
  {
    auto [high_word, low_word] = argument;
    return client.holding_registers(modbus_address(m_can_id, index), 2) = {high_word, low_word};
  }
}  // namespace boarai::hardware::roboteq