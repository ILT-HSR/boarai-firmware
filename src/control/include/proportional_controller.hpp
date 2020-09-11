#ifndef BOARAI_CONTROL_PROPORTIONAL_CONTROLLER_HPP
#define BOARAI_CONTROL_PROPORTIONAL_CONTROLLER_HPP

namespace boarai::control
{
  template<typename T>
  struct proportional_controller
  {
    proportional_controller() = default;

    proportional_controller(double proportional_factor)
        : proportional_factor{proportional_factor} {};

    auto set_proportional_factor(T proportional_factor) -> void
    {
      this->proportional_factor = proportional_factor;
    }

    auto operator()(T current_value, T target_value) -> T
    {
      auto current_error = target_value - current_value;
      return proportional_factor * current_error;
    };

  private:
    double proportional_factor;
  };
}  // namespace boarai::control

#endif