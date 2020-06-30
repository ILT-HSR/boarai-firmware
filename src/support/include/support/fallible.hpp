#ifndef BOARAI_SUPPORT_FALLIBLE_HPP
#define BOARAI_SUPPORT_FALLIBLE_HPP

#include <optional>
#include <system_error>
#include <variant>

namespace boarai
{

  template<typename SuccessType>
  using fallible = std::variant<std::error_code, SuccessType>;

  template<typename SuccessType>
  auto inline constexpr get_error(fallible<SuccessType> const & result) -> std::error_code
  {
    return std::get<std::error_code>(result);
  }

  template<typename SuccessType>
  auto inline constexpr throw_error(fallible<SuccessType> const & result) -> void
  {
    if (std::holds_alternative<std::error_code>(result))
    {
      throw std::get<std::error_code>(result);
    }
  }

  template<typename SuccessType>
  auto inline constexpr expect(fallible<SuccessType> const & result) -> SuccessType
  {
    if (std::holds_alternative<SuccessType>(result))
    {
      return std::get<SuccessType>(result);
    }
    throw std::get<std::error_code>(result);
  }

  template<typename SuccessType>
  auto inline constexpr expect(std::nothrow_t, fallible<SuccessType> const & result) -> std::optional<SuccessType>
  {
    if (std::holds_alternative<SuccessType>(result))
    {
      return std::get<SuccessType>(result);
    }
    return std::nullopt;
  }

}  // namespace boarai

#endif