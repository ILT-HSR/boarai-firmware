#include "rclcpp/node_impl.hpp"
#include "support/enum_utility.hpp"
#include "support/to_string.hpp"
#include "tank_drive_debug/tank_drive_debug.hpp"

#include <array>
#include <type_traits>
#include <utility>

auto const default_wheel_spacing{0.0};
auto const default_maximum_linear_velocity{1.0};
auto const default_acceleration_delay{0.1};

namespace boarai::hardware
{
  auto constexpr parameter_names = std::array{
      std::pair{tank_drive_debug::parameter::wheel_spacing, "wheel_spacing"},
      std::pair{tank_drive_debug::parameter::maximum_linear_velocity, "maximum_linear_velocity"},
      std::pair{tank_drive_debug::parameter::acceleration_delay, "acceleration_delay"},
  };

  auto tank_drive_debug::declare_parameters() -> void
  {
    declare_parameter(to_string(parameter::wheel_spacing), default_wheel_spacing);
    declare_parameter(to_string(parameter::maximum_linear_velocity), default_maximum_linear_velocity);
    declare_parameter(to_string(parameter::acceleration_delay), default_acceleration_delay);
  }

  auto tank_drive_debug::wheel_spacing() -> double
  {
    auto result{0.0};
    get_parameter_or(to_string(parameter::wheel_spacing), result, default_wheel_spacing);
    return result;
  }

  auto tank_drive_debug::maximum_linear_velocity() -> double
  {
    auto result{0.0};
    get_parameter_or(to_string(parameter::maximum_linear_velocity), result, default_maximum_linear_velocity);
    return result;
  }

  auto tank_drive_debug::acceleration_delay() -> double
  {
    auto result{0.0};
    get_parameter_or(to_string(parameter::acceleration_delay), result, default_acceleration_delay);
    return result;
  }
}  // namespace boarai::hardware

namespace boarai
{
  using namespace hardware;

  template<>
  auto to_string(tank_drive_debug::parameter const & object) -> std::string
  {
    assert(is_valid<tank_drive::parameter>(static_cast<std::underlying_type_t<tank_drive::parameter>>(object)));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.first == object;
    });

    return found->second;
  }

  template<>
  auto from_string(std::string const & stringified) -> hardware::tank_drive_debug::parameter
  {
    assert(is_valid<tank_drive::parameter>(stringified));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.second == stringified;
    });

    return found->first;
  }

  template<>
  auto is_valid<hardware::tank_drive_debug::parameter>(std::underlying_type_t<hardware::tank_drive_debug::parameter> candidate)
      -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }

  template<>
  auto is_valid<hardware::tank_drive_debug::parameter>(std::string const & candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }
}  // namespace boarai