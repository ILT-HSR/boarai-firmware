#include "modbuscpp/modbuscpp.hpp"
#include "modbuscpp/test_server.hpp"

#include <cute/cute.h>
#include <cute/cute_runner.h>
#include <cute/tap_listener.h>

#include <chrono>
#include <future>
#include <iostream>
#include <string>
#include <utility>
#include <variant>

using test_server = boarai::hardware::test::test_server;

auto constexpr default_mapping_parameters = test_server::mapping_parameters{
    9999,  // coils
    9999,  // discrete inputs
    9999,  // holding registers
    9999,  // input registers
};

auto constexpr default_server_port = 8989;

auto test_modbus_tcp_can_connect_if_server_is_listening()
{
  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};

  connection.close();
  ASSERT_EQUAL(std::future_status::ready, runner.wait_for(std::chrono::milliseconds{500}));
}

auto test_modbus_tcp_connect_throws_if_no_server_is_listening()
{
  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  ASSERT_THROWS(modbus::connection{std::move(context)}, std::system_error);
}

auto test_modbus_tcp_can_read_single_coil()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  ASSERT(std::holds_alternative<bool>(*client.coil(1337_addr)));
}

auto test_modbus_tcp_can_read_multiple_coils()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  ASSERT(std::holds_alternative<std::vector<bool>>(*client.coils(1337_addr, 3)));
}

auto test_modbus_tcp_fails_when_trying_to_read_too_many_coils()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  ASSERT(std::holds_alternative<std::error_code>(*client.coils(1337_addr, MODBUS_MAX_READ_BITS + 1)));
}

auto test_modbus_tcp_fails_when_trying_to_write_too_many_coils()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  auto data = std::vector<bool>(MODBUS_MAX_READ_BITS + 1);

  ASSERT((client.coils(1337_addr, MODBUS_MAX_READ_BITS + 1) = data));
}

auto test_modbus_tcp_fails_when_trying_to_read_invalid_coil()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  ASSERT(std::holds_alternative<std::error_code>(*client.coil(0xcafe_addr)));
}

auto test_modbus_tcp_fails_when_trying_to_write_invalid_coil()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  ASSERT((client.coil(0xcafe_addr) = true));
}

auto test_modbus_tcp_can_write_single_coil()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  ASSERT_EQUAL(std::error_code{}, (client.coil(42_addr) = true));
}

auto test_modbus_tcp_can_write_and_read_back_single_coil()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  client.coil(42_addr) = true;

  ASSERT_EQUAL(true, static_cast<bool>(client.coil(42_addr)));
}

auto test_modbus_tcp_can_write_and_read_back_multiple_coils()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  client.coils(18_addr, 5) = {true, true, false, true, false};

  ASSERT_EQUAL((std::vector{true, true, false, true, false}), static_cast<std::vector<bool>>(client.coils(18_addr, 5)));
}

auto test_modbus_tcp_fails_when_number_of_data_points_is_smaller_than_coil_count_when_writing()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  ASSERT_EQUAL(make_error_code(std::errc::invalid_argument), (client.coils(18_addr, 5) = {true, true}));
}

auto test_modbus_tcp_fails_when_number_of_data_points_is_larger_than_coil_count_when_writing()
{
  using namespace modbus::modbus_literals;

  auto server = test_server{default_mapping_parameters, default_server_port};
  auto runner = std::async(std::launch::async, [&] {
    server.run();
  });

  auto context = modbus::tcp_context{"127.0.0.1", default_server_port};
  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  ASSERT_EQUAL(make_error_code(std::errc::invalid_argument), (client.coils(18_addr, 3) = {true, true, false, true}));
}

auto modbuscpp_suite() -> std::pair<cute::suite, std::string>
{
  // clang-format off
  return {
      cute::suite{
          CUTE(test_modbus_tcp_can_connect_if_server_is_listening),
          CUTE(test_modbus_tcp_connect_throws_if_no_server_is_listening),
          CUTE(test_modbus_tcp_can_read_single_coil),
          CUTE(test_modbus_tcp_can_read_multiple_coils),
          CUTE(test_modbus_tcp_fails_when_trying_to_read_too_many_coils),
          CUTE(test_modbus_tcp_fails_when_trying_to_write_too_many_coils),
          CUTE(test_modbus_tcp_fails_when_trying_to_read_invalid_coil),
          CUTE(test_modbus_tcp_fails_when_trying_to_write_invalid_coil),
          CUTE(test_modbus_tcp_can_write_single_coil),
          CUTE(test_modbus_tcp_can_write_and_read_back_single_coil),
          CUTE(test_modbus_tcp_can_write_and_read_back_multiple_coils),
          CUTE(test_modbus_tcp_fails_when_number_of_data_points_is_smaller_than_coil_count_when_writing),
          CUTE(test_modbus_tcp_fails_when_number_of_data_points_is_larger_than_coil_count_when_writing),
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