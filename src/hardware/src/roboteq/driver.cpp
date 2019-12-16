#include "roboteq/driver.hpp"

#include "roboteq/channel.hpp"
#include "roboteq/commands.hpp"

#include <modbuscpp/client.hpp>

#include <algorithm>
#include <iterator>
#include <system_error>
#include <type_traits>
#include <utility>
#include <variant>

namespace boarai::hardware::roboteq
{
  namespace
  {
    /**
     * @internal
     *
     * Check that a commands takes arguments
     *
     * @param command The command to check
     */
    auto command_takes_arguments(command command) -> bool
    {
      return command.argument_type != command_argument_type::none;
    }

    /**
     * @internal
     *
     * Determine the size of a command's argument
     */
    auto argument_size(command command) -> int
    {
      switch (command.argument_type)
      {
      case command_argument_type::boolean:
      case command_argument_type::signed_8:
      case command_argument_type::unsigned_8:
        return 1;
      case command_argument_type::signed_16:
      case command_argument_type::unsigned_16:
        return 2;
      case command_argument_type::signed_32:
      case command_argument_type::unsigned_32:
        return 4;
      default:
        return 0;
      }
    }

    /**
     * @internal
     *
     * Encode the argument of a command into a pair 16-bit unsigned integers
     */
    template<typename ArgumentType>
    auto encode_command_argument(command command, ArgumentType argument)
        -> std::variant<std::pair<std::uint16_t, std::uint16_t>, std::error_code>
    {
      if (argument_size(command) != sizeof(argument))
      {
        return make_error_code(std::errc::invalid_argument);
      }

      auto high_word = static_cast<std::uint16_t>((static_cast<std::uint32_t>(argument) >> 16) & 0xff);
      auto low_word = static_cast<std::uint16_t>(static_cast<std::uint32_t>(argument) & 0xff);
      return std::pair{high_word, low_word};
    }

    /**
     * @internal
     *
     * Run a controller command that takes no arguments
     *
     * @param client The MODBUS client to run the command through
     * @param command The command to run
     */
    auto execute(modbus::client & client, command command) -> std::error_code
    {
      if (command_takes_arguments(command))
      {
        return make_error_code(std::errc::invalid_argument);
      }

      return client.holding_registers(static_cast<modbus::address>(command.id), 2) = {0, 0};
    }

    /**
     * @internal
     *
     * Run a controller command that takes an argument
     *
     * @param client The MODBUS client to run the command through
     * @param command The command to run
     * @param argument The argument for the command invocation
     */
    template<typename ArgumentType>
    auto execute(modbus::client & client, command command, ArgumentType argument) -> std::error_code
    {
      if (!command_takes_arguments(command))
      {
        return make_error_code(std::errc::invalid_argument);
      }

      return std::visit(
          [&](auto data) {
            if constexpr (std::is_same_v<std::error_code, decltype(data)>)
            {
              return data;
            }
            else
            {
              auto [high, low] = data;
              return client.holding_registers(static_cast<modbus::address>(command.id), 2) = {high, low};
            }
          },
          encode_command_argument(command, argument));
    }

    /**
     * @internal
     *
     * Run a controller command that takes an argument
     *
     * @param client The MODBUS client to run the command through
     * @param command The command to run
     * @param channel The channel to run the command on
     * @param argument The argument for the command invocation
     */
    template<typename ArgumentType>
    auto execute(modbus::client & client, command command, channel channel, ArgumentType argument) -> std::error_code
    {
      auto channeled_command =
          roboteq::command{static_cast<std::uint16_t>(command.id + static_cast<std::uint16_t>(channel)), command.argument_type};
      return execute(client, channeled_command, argument);
    }

  }  // namespace

  driver::driver(modbus::client & client)
      : m_client{client}
  {
  }

  auto driver::emergency_stop() -> std::error_code
  {
    return execute(m_client, commands::emergency_stop);
  }

  auto driver::go_to_position(channel channel, position position) -> std::error_code
  {
    auto command =
        position.type() == position_type::absolute ? commands::go_to_absolute_position : commands::go_to_relative_position;
    return execute(m_client, command, channel, position.encoder_count());
  }

  auto driver::set_motor_speed(channel channel, std::int32_t speed) -> std::error_code
  {
    return execute(m_client, commands::set_motor_speed, channel, speed);
  }

  auto driver::set_encoder_counter(channel channel, std::int32_t counts) -> std::error_code
  {
    return execute(m_client, commands::set_encoder_counter, channel, counts);
  }

  auto driver::set_hall_counter(channel channel, std::int32_t counts) -> std::error_code
  {
    return execute(m_client, commands::set_hall_counter, channel, counts);
  }

}  // namespace boarai::hardware::roboteq
