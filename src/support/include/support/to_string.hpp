#ifndef BOARAI_SUPPORT_TO_STRING_HPP
#define BOARAI_SUPPORT_TO_STRING_HPP

#include <string>

namespace boarai
{

  /**
   * Convert an instance of SourceType to a string
   */
  template<typename SourceType>
  auto to_string(SourceType const & object) -> std::string;

  /**
   * Convert a string to an instance of TargetType
   */
  template<typename TargetType>
  auto from_string(std::string const & stringified) -> TargetType;

}  // namespace boarai

#endif