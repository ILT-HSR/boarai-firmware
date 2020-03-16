#ifndef BOARAI_SUPPORT_STRING_UTILITY_HPP
#define BOARAI_SUPPORT_STRING_UTILITY_HPP

#include <string>
#include <utility>

namespace boarai
{

  template<typename JointType, typename FirstPart, typename... Rest>
  auto join(JointType && joint, FirstPart && first, Rest &&... remaining) -> std::string
  {
    return static_cast<std::string>(first) + (static_cast<std::string>(joint) + ... + static_cast<std::string>(remaining));
  }

}  // namespace boarai

#endif