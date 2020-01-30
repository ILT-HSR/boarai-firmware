#ifndef BOARAI_SUPPORT_TO_STRING_HPP
#define BOARAI_SUPPORT_TO_STRING_HPP

#include <string>

namespace boarai
{

  template<typename SourceType>
  auto to_string(SourceType const & object) -> std::string;

  template<typename TargetType>
  auto from_string(std::string const & stringified) -> TargetType;

}  // namespace boarai

#endif