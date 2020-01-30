#ifndef BOARAI_HARDWARE_ROBOTEQ_POSITION_HPP
#define BOARAI_HARDWARE_ROBOTEQ_POSITION_HPP

#include <cassert>
#include <cstdint>
#include <limits>

namespace boarai::hardware::roboteq
{
  enum struct position_type : bool
  {
    absolute,
    relative,
  };

  struct position
  {
    constexpr position(std::int32_t encoder_count, position_type type)
        : m_encoder_count{encoder_count}
        , m_type{type}
    {
    }

    auto constexpr encoder_count() const noexcept -> std::int32_t
    {
      return m_encoder_count;
    }

    auto constexpr type() const noexcept -> position_type
    {
      return m_type;
    }

    auto constexpr operator-() const noexcept -> position
    {
      return {-m_encoder_count, m_type};
    }

  private:
    std::int32_t m_encoder_count;
    position_type m_type;
  };

  inline namespace literals
  {
    inline namespace position_literals
    {
      auto constexpr operator""_abs(unsigned long long encoder_count) -> position
      {
        assert(static_cast<std::int32_t>(encoder_count) <= std::numeric_limits<std::int32_t>::max());
        return {static_cast<std::int32_t>(encoder_count), position_type::absolute};
      }

      auto constexpr operator""_rel(unsigned long long encoder_count) -> position
      {
        assert(static_cast<std::int32_t>(encoder_count) <= std::numeric_limits<std::int32_t>::max());
        return {static_cast<std::int32_t>(encoder_count), position_type::relative};
      }
    }  // namespace position_literals
  }    // namespace literals
}  // namespace boarai::hardware::roboteq

#endif