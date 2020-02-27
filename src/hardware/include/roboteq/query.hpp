#ifndef BOARAI_HARDWARE_ROBOTEQ_QUERY_HPP
#define BOARAI_HARDWARE_ROBOTEQ_QUERY_HPP

#include "modbuscpp/client.hpp"
#include "roboteq/modbus_address.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <system_error>
#include <utility>
#include <variant>

namespace boarai::hardware::roboteq
{

  namespace impl
  {
    template<typename ResultType, std::size_t... Indices>
    auto to(std::array<std::byte, 4> const & bytes, std::index_sequence<Indices...>)
    {
      return ((static_cast<ResultType>(bytes[3 - Indices]) << Indices * 8) | ...);
    }
  }  // namespace impl

  template<typename ResultType>
  struct query
  {
    static_assert(sizeof(ResultType) <= 4 && sizeof(ResultType) >= 1);

    explicit constexpr query(std::uint16_t can_id) noexcept
        : m_can_id{can_id}
    {
    }

    auto operator()(modbus::client & client, std::uint16_t index = 0) const -> std::variant<ResultType, std::error_code>
    {
      auto response = *client.input_registers(modbus_address(m_can_id, index), 2);
      if (std::holds_alternative<std::error_code>(response))
      {
        return std::get<std::error_code>(response);
      }

      auto read_value = std::get<0>(response);
      auto bytes = std::array{
          static_cast<std::byte>(read_value[0].to_ulong() >> 8 & 0xff),
          static_cast<std::byte>(read_value[0].to_ulong() & 0xff),
          static_cast<std::byte>(read_value[1].to_ulong() >> 8 & 0xff),
          static_cast<std::byte>(read_value[1].to_ulong() & 0xff),
      };

      return impl::to<ResultType>(bytes, std::make_index_sequence<sizeof(ResultType)>{});
    }

  private:
    std::uint16_t m_can_id;
  };

}  // namespace boarai::hardware::roboteq

#endif