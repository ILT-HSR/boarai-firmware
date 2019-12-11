#include <cute/cute.h>
#include <cute/cute_runner.h>
#include <cute/tap_listener.h>

#include <iostream>
#include <string>
#include <utility>

auto test_start_writing_tests()
{
  ASSERTM("Start writing tests!", false);
}

auto modbuscpp_suite() -> std::pair<cute::suite, std::string>
{
  // clang-format off
  return {
      cute::suite{
          CUTE(test_start_writing_tests),
      },
      "MODBUSCPP Unit Tests"
  };
  // clang-format on
}

int main(int argc, char const * const * argv)
{
  auto listener = cute::tap_listener<>{std::cout};
  auto runner = cute::makeRunner(listener, argc, argv);
  auto [suite, name] = modbuscpp_suite();
  return !runner(suite, name.c_str());
}