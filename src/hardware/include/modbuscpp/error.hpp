#ifndef BOARAI_HARDWARE_MODBUSCPP_ERROR_HPP
#define BOARAI_HARDWARE_MODBUSCPP_ERROR_HPP

#include <string>
#include <system_error>
#include <type_traits>
#include <variant>

namespace modbus
{
  enum modbus_error
  {
    invalid_slave_number = 1,
    invalid_context,
    invalid_duration,
    invalid_ip_address,
    invalid_serial_mode,
    invalid_rts_mode,
    invalid_rts_delay,
    too_many_coils_requested,
    too_many_discrete_inputs_requested,
    too_many_holding_registers_requested,
    too_many_input_registers_requested,
  };

  struct modbus_category_impl : std::error_category
  {
    char const * name() const noexcept override final;

    std::string message(int code) const override final;

    std::error_condition default_error_condition(int code) const noexcept override final;
  };

  auto modbus_category() -> std::error_category const &;

  auto make_error_code(modbus_error error_code) -> std::error_code;

  auto make_error_condition(modbus_error error_code) -> std::error_condition;

}  // namespace modbus

namespace std
{
  template<>
  struct is_error_code_enum<modbus::modbus_error> : true_type
  {
  };
}  // namespace std

template<typename SuccessType>
auto constexpr operator!(std::variant<SuccessType, std::error_code> const & failable) -> bool
{
  return std::holds_alternative<std::error_code>(failable);
}

template<typename SuccessType>
auto constexpr operator*(std::variant<SuccessType, std::error_code> const & failable) -> SuccessType const &
{
  return std::get<SuccessType>(failable);
}

template<typename SuccessType>
auto constexpr operator*(std::variant<SuccessType, std::error_code> & failable) -> SuccessType &
{
  return std::get<SuccessType>(failable);
}

template<typename SuccessType>
auto constexpr operator*(std::variant<SuccessType, std::error_code> && failable) -> SuccessType &&
{
  return std::get<SuccessType>(failable);
}

template<typename SuccessType>
auto constexpr operator*(std::variant<SuccessType, std::error_code> const && failable) -> SuccessType const &&
{
  return std::get<SuccessType>(failable);
}

#endif