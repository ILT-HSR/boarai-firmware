#ifndef BOARAI_SUPPORT_STRING_UTILITY_HPP
#define BOARAI_SUPPORT_STRING_UTILITY_HPP

#include <sstream>
#include <string>
#include <utility>

namespace boarai
{

  template<typename JointType, typename... Parts>
  auto join(JointType && joint, Parts &&... parts) -> std::string
  {
    auto joined{std::ostringstream{}};
    auto part_count{sizeof...(parts)};
    auto part_index{decltype(part_count){}};
    ((joined << parts << (++part_index < part_count ? joint : "")), ...);
    return joined.str();
  }

  // (first ? joint : ((first = true), ""))

}  // namespace boarai

#endif