#ifndef BOARAI_SUPPORT_ENUM_UTILITY_HPP
#define BOARAI_SUPPORT_ENUM_UTILITY_HPP

#include <algorithm>
#include <array>
#include <cstddef>
#include <iterator>
#include <string>
#include <type_traits>
#include <utility>

namespace boarai
{

  namespace impl
  {
    auto constexpr equals(char const * lhs, char const * rhs)
    {
      if ((lhs && !rhs) || (!lhs && rhs))
      {
        return false;
      }

      while (*lhs && *rhs)
      {
        if (*lhs != *rhs)
        {
          return false;
        }
        ++lhs, ++rhs;
      }
      return *lhs == *rhs;
    }
  }  // namespace impl

  /**
   * Check if all entries in the enum map are unique
   *
   * @return true iff. no key nor any value is duplicatend in the map
   */
  template<typename KeyType, typename ValueType, std::size_t Size>
  auto constexpr enum_mappings_are_unique(std::array<std::pair<KeyType, ValueType>, Size> const & map) -> bool
  {
    static_assert(std::is_enum_v<KeyType>, "The key type of the map must be an enum!");

    if constexpr (!Size)
    {
      return true;
    }

    for (auto const & entryToCheck : map)
    {
      auto enumOccurrences{0};
      for (auto const & entry : map)
      {
        enumOccurrences += static_cast<decltype(enumOccurrences)>(entryToCheck.first == entry.first);
      }
      if (enumOccurrences > 1)
      {
        return false;
      }
    }

    for (auto const & entryToCheck : map)
    {
      auto nameOccurrences{0};
      for (auto const & entry : map)
      {
        if constexpr (std::is_same_v<char const *, std::remove_cv_t<ValueType>>)
        {
          nameOccurrences += static_cast<decltype(nameOccurrences)>(impl::equals(entryToCheck.second, entry.second));
        }
        else
        {
          nameOccurrences += static_cast<decltype(nameOccurrences)>(entryToCheck.second == entry.second);
        }
      }
      if (nameOccurrences > 1)
      {
        return false;
      }
    }

    return true;
  }

  /**
   * Check if the enum map contains entries for all enumerators
   *
   * This function expectes that:
   *
   *   - all enumerators are defined to be consecutive
   *   - the enum type has an enumerator END_OF_ENUM as its last entry
   *
   * @param first The first enumerator of the enum
   */
  template<typename KeyType, typename ValueType, std::size_t Size>
  auto constexpr enum_map_has_all_entries(std::array<std::pair<KeyType, ValueType>, Size> const &, KeyType first) -> bool
  {
    static_assert(std::is_enum_v<KeyType>, "The key type of the map must be an enum!");

    auto beginValue = static_cast<std::underlying_type_t<KeyType>>(first);
    auto endValue = static_cast<std::underlying_type_t<KeyType>>(KeyType::END_OF_ENUM);

    return Size == (endValue - beginValue);
  }

  template<typename EnumType>
  auto is_valid(std::underlying_type_t<EnumType> candidate) -> bool;

  template<typename EnumType>
  auto is_valid(std::string const & candidate) -> bool;

  template<typename EnumType, typename ValueType, std::size_t MapSize>
  auto is_valid_helper(std::underlying_type_t<EnumType> candidate,
                       std::array<std::pair<EnumType, ValueType>, MapSize> const & map) -> bool
  {
    static_assert(std::is_enum_v<EnumType>, "The key type of the map must be an enum!");
    auto found = std::find_if(cbegin(map), cend(map), [&](auto entry) {
      return candidate == static_cast<std::underlying_type_t<EnumType>>(entry.first);
    });
    return found != cend(map);
  }

  template<typename EnumType, typename ValueType, std::size_t MapSize>
  auto is_valid_helper(std::string candidate, std::array<std::pair<EnumType, ValueType>, MapSize> const & map) -> bool
  {
    static_assert(std::is_enum_v<EnumType>, "The key type of the map must be an enum!");
    auto found = std::find_if(cbegin(map), cend(map), [&](auto entry) {
      return candidate == entry.second;
    });
    return found != cend(map);
  }

}  // namespace boarai

#endif