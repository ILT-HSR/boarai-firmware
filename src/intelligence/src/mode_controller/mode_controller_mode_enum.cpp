#include "mode_controller/mode_controller.hpp"
#include "support/enum_utility.hpp"
#include "support/to_string.hpp"

namespace boarai
{
  using namespace intelligence;

  auto constexpr mode_names = std::array{
      std::pair{mode_controller::mode::none, "none"},
      std::pair{mode_controller::mode::manual, "manual"},
      std::pair{mode_controller::mode::autonomous, "autonomous"},
  };

  static_assert(enum_mappings_are_unique(mode_names), "duplicate key or value in mode mappings");
  static_assert(enum_map_has_all_entries(mode_names, mode_controller::mode::none), "missing mapping for mode");

  template<>
  auto to_string(mode_controller::mode const & object) -> std::string
  {
    assert(is_valid<mode_controller::mode>(static_cast<std::underlying_type_t<mode_controller::mode>>(object)));
    auto found = std::find_if(cbegin(mode_names), cend(mode_names), [&](auto candidate) {
      return candidate.first == object;
    });

    return found->second;
  }

  template<>
  auto from_string(std::string const & stringified) -> mode_controller::mode
  {
    assert(is_valid<tank_drive::parameter>(stringified));
    auto found = std::find_if(cbegin(mode_names), cend(mode_names), [&](auto candidate) {
      return candidate.second == stringified;
    });

    return found->first;
  }

  template<>
  auto is_valid<mode_controller::mode>(std::underlying_type_t<mode_controller::mode> candidate) -> bool
  {
    return is_valid_helper(candidate, mode_names);
  }

  template<>
  auto is_valid<mode_controller::mode>(std::string const & candidate) -> bool
  {
    return is_valid_helper(candidate, mode_names);
  }
}  // namespace boarai
