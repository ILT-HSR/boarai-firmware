#ifndef BOARAI_HARDWARE_ROBOTEQ_COMMAND_HPP
#define BOARAI_HARDWARE_ROBOTEQ_COMMAND_HPP

#include "modbuscpp/address.hpp"
#include "modbuscpp/client.hpp"

#include <algorithm>
#include <array>
#include <cstdint>
#include <string>
#include <system_error>
#include <utility>
#include <variant>

namespace boarai::hardware::roboteq
{
  enum struct command_error
  {
    missing_argument = 1,
    unexpected_argument,
    argument_type_mismatch,
  };

  struct command_category_impl : std::error_category
  {
    char const * name() const noexcept;

    std::string message(int error) const;

    std::error_condition default_error_condition(int error) const noexcept;
  };

  auto command_category() -> std::error_category const &;

  auto make_error_code(command_error code) -> std::error_code;

  auto make_error_condition(command_error code) -> std::error_condition;

  /**
   * A Roboteq Motor Driver command
   */
  struct command
  {
    /**
     * An enumeration type to represent the different kinds of arguments a command might take
     */
    enum struct argument_type
    {
      none,
      boolean,
      signed_8,
      unsigned_8,
      signed_16,
      unsigned_16,
      signed_32,
      unsigned_32,
    };

    /**
     * Construct a command based on a CANOpen ID and an argument type
     *
     * @param can_id The CANOpen ID of the command
     * @param argument_type The type of the command's argument
     */
    constexpr command(std::uint16_t can_id, argument_type argument_type) noexcept
        : m_can_id{can_id}
        , m_argument_type{argument_type}
    {
    }

    /**
     * Execute this command
     *
     * @param client A MODBUS client interface for a motor driver node
     * @param index The index of the command (defaults to 0)
     * @return An error code reflecting whether the command was successful or not
     */
    auto operator()(modbus::client & client, std::uint16_t index = 0) const -> std::error_code;

    /**
     * Execute this command
     *
     * @param client A MODBUS client interface for a motor driver node
     * @param argument The argument for the command
     * @param index The index of the command (defaults to 0)
     * @return An error code reflecting whether the command was successful or not
     */
    template<typename ArgumentType>
    auto operator()(modbus::client & client, ArgumentType argument, std::uint16_t index = 0) const -> std::error_code
    {
      if (!takes_arguments())
      {
        return make_error_code(command_error::unexpected_argument);
      }

      return std::visit(
          [&](auto result) {
            if constexpr (std::is_same_v<std::error_code, decltype(result)>)
            {
              return result;
            }
            else
            {
              return do_execute(client, result, index);
            }
          },
          encode_argument(argument));
    }

  private:
    /**
     * @internal
     *
     * Execute a command on a MODBUS node
     *
     * @param client A MODBUS client interface for a motor driver node
     * @param argument The argument for the command
     * @param channel The index of the command
     */
    auto do_execute(modbus::client & client, std::pair<std::uint16_t, std::uint16_t> argument, std::uint16_t index) const
        -> std::error_code;

    /**
     * @internal
     *
     * Calculate the MODBUS address of the command for a given index
     *
     * @param index The index of the command (e.g. a motor channel)
     * @return The indexed MODBUS address of the command
     */
    auto constexpr modbus_address(std::uint16_t index) const noexcept -> modbus::address
    {
      auto base = static_cast<std::uint32_t>(m_can_id) << 5;
      auto indexed = base | index;
      return modbus::address{static_cast<std::uint16_t>(indexed & 0xffff)};
    }

    /**
     * @internal
     *
     * Check if the command requires an argument
     *
     * @return @p true iff. the command requires an argument, @p false otherwise
     */
    auto constexpr takes_arguments() const noexcept -> bool
    {
      return m_argument_type != argument_type::none;
    }

    /**
     * @internal
     *
     * Get the command's argument size
     *
     * @return The size of the command's argument
     */
    auto constexpr argument_size() const noexcept -> int
    {
      switch (m_argument_type)
      {
      case argument_type::boolean:
      case argument_type::signed_8:
      case argument_type::unsigned_8:
        return 1;
      case argument_type::signed_16:
      case argument_type::unsigned_16:
        return 2;
      case argument_type::signed_32:
      case argument_type::unsigned_32:
        return 4;
      default:
        return 0;
      }
    }

    /**
     * @internal
     *
     * Encode a value into a pair of 16-bit unsigned integers
     *
     * @param argument The value to encode
     */
    template<typename ArgumentType>
    auto encode_argument(ArgumentType argument) const -> std::variant<std::pair<std::uint16_t, std::uint16_t>, std::error_code>
    {
      if (argument_size() != sizeof(argument))
      {
        return make_error_code(std::errc::invalid_argument);
      }

      auto converted_argument = static_cast<std::uint32_t>(argument);
      auto high_word = static_cast<std::uint16_t>((converted_argument >> 16) & 0xffff);
      auto low_word = static_cast<std::uint16_t>(converted_argument & 0xffff);
      return std::pair{high_word, low_word};
    }

    std::uint16_t m_can_id;
    argument_type m_argument_type;
  };
}  // namespace boarai::hardware::roboteq

namespace std
{
  template<>
  struct is_error_code_enum<boarai::hardware::roboteq::command_error> : true_type
  {
  };
}  // namespace std

#endif
