#ifndef BOARAI_SUPPORT_LERP_HPP
#define BOARAI_SUPPORT_LERP_HPP

namespace boarai
{
  constexpr double lerp(double a, double b, double t) noexcept
  {
    return a + t * (b - a);
  }
}  // namespace boarai

#endif