#include "support/string_utility.hpp"

#include <cute/cute.h>
#include <cute/cute_runner.h>
#include <cute/tap_listener.h>

#include <string>

auto test_joining_two_strings_does_not_include_a_trailing_joint()
{
  auto joined = boarai::join("/", "hello", "world");
  ASSERT_EQUAL("hello/world", joined);
}

auto test_joining_a_single_string_preserves_the_trailing_joint()
{
  auto joined = boarai::join("/", "hello");
  ASSERT_EQUAL("hello/", joined);
}

auto support_suite() -> std::pair<cute::suite, std::string>
{
  return {
      cute::suite{
          CUTE(test_joining_two_strings_does_not_include_a_trailing_joint),
          CUTE(test_joining_a_single_string_preserves_the_trailing_joint),
      },
      "Support Library Unit Tests",
  };
}

int main(int argc, char const * const * argv)
{
  auto listener = cute::tap_listener<>{std::cout};
  auto runner = cute::makeRunner(listener, argc, argv);
  auto [suite, name] = support_suite();
  return !runner(suite, name.c_str());
}