#include "rclcpp/node_impl.hpp"
#include "support/enum_utility.hpp"
#include "support/to_string.hpp"
#include "velocity_control/velocity_control.hpp"

#include <algorithm>
#include <array>
#include <string>
#include <type_traits>
#include <utility>

auto const default_proportional_factor{1.0};

namespace boarai::control
{
  auto constexpr parameter_names = std::array{
      std::pair{velocity_control::parameter::proportional_factor, "proportional_factor"},
  };

  auto velocity_control::declare_parameters() -> void
  {
    declare_parameter(to_string(parameter::proportional_factor), default_proportional_factor);
  }

  auto velocity_control::proportional_factor() -> double
  {
    auto result{1.0};
    get_parameter_or(to_string(parameter::proportional_factor), result, default_proportional_factor);
    return result;
  }

  auto velocity_control::on_parameters_changed(std::vector<rclcpp::Parameter> new_parameters)
      -> rcl_interfaces::msg::SetParametersResult
  {
    auto result = rcl_interfaces::msg::SetParametersResult{};
    result.successful = true;

    for (auto & param : new_parameters)
    {
      auto name = param.get_name();

      if (!is_valid<velocity_control::parameter>(name))
      {
        log_warning("received change of unknown parameter '{}'", name);
        result.successful = false;
        return result;
      }

      switch (from_string<velocity_control::parameter>(name))
      {
      case parameter::proportional_factor:
        result.successful &= on_proportional_factor_changed(param.as_double());
        break;
      case parameter::END_OF_ENUM:
        break;
      };
    }

    return result;
  }

  auto velocity_control::on_proportional_factor_changed(double new_value) -> bool
  {
    m_controller_proportional.set_proportional_factor(new_value);
    return true;
  }
}  // namespace boarai::control

namespace boarai
{
  using namespace control;

  template<>
  auto to_string(velocity_control::parameter const & object) -> std::string
  {
    assert(is_valid<velocity_control::parameter>(static_cast<std::underlying_type_t<velocity_control::parameter>>(object)));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.first == object;
    });

    return found->second;
  }

  template<>
  auto from_string(std::string const & stringified) -> velocity_control::parameter
  {
    assert(is_valid<velocity_control::parameter>(stringified));
    auto found = std::find_if(cbegin(parameter_names), cend(parameter_names), [&](auto candidate) {
      return candidate.second == stringified;
    });

    return found->first;
  }

  template<>
  auto is_valid<velocity_control::parameter>(std::underlying_type_t<velocity_control::parameter> candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }

  template<>
  auto is_valid<velocity_control::parameter>(std::string const & candidate) -> bool
  {
    return is_valid_helper(candidate, parameter_names);
  }
}  // namespace boarai