#include "support/enum_utility.hpp"
#include "support/to_string.hpp"
#include "tank_drive/tank_drive.hpp"

namespace boarai
{
  using namespace hardware;

  auto constexpr parameter_names = std::array{
      std::pair{tank_drive::parameter::driver_address, "driver_address"},
      std::pair{tank_drive::parameter::driver_port, "driver_port"},
      std::pair{tank_drive::parameter::driver_enabled, "driver_enabled"},
      std::pair{tank_drive::parameter::maximum_linear_velocity, "maximum_linear_velocity"},
      std::pair{tank_drive::parameter::maximum_angular_velocity, "maximum_angular_velocity"},
  };

  static_assert(enum_mappings_are_unique(parameter_names), "missing mapping for parameter");
  static_assert(enum_map_has_all_entries(parameter_names, tank_drive::parameter::driver_address),
                "duplicate key or value in parameter mappings");

  template<>
  auto to_string(tank_drive::parameter const & object) -> std::string
  {
    assert(is_valid<tank_drive::parameter>(static_cast<std::underlying_type_t<tank_drive::parameter>>(object)));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.first == object;
    });

    return found->second;
  }

  template<>
  auto from_string(std::string const & stringified) -> hardware::tank_drive::parameter
  {
    assert(is_valid<tank_drive::parameter>(stringified));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.second == stringified;
    });

    return found->first;
  }

  template<>
  auto is_valid<hardware::tank_drive::parameter>(std::underlying_type_t<hardware::tank_drive::parameter> candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }

  template<>
  auto is_valid<hardware::tank_drive::parameter>(std::string const & candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }
}  // namespace boarai
