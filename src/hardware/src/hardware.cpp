#include "modbuscpp/modbuscpp.hpp"
#include "rclcpp/rclcpp.hpp"

#include <chrono>
#include <exception>
#include <iostream>
#include <thread>
#include <utility>

int main()
try
{
  using namespace std::chrono_literals;
  using namespace modbus::modbus_literals;

  auto context = modbus::tcp_context{"192.168.1.20", 502};

  context.response_timeout(1s);
  context.slave_id(1);

  auto connection = modbus::connection{std::move(context)};
  auto client = modbus::client{connection};

  std::cout << (client.holding_registers(0x0001_addr, 2) = {0x0000, 0x01f4}).message() << '\n';
  std::this_thread::sleep_for(1500ms);
  std::cout << (client.holding_registers(0x0002_addr, 2) = {0x0000, 0x0100}).message() << '\n';
  std::this_thread::sleep_for(1500ms);
  std::cout << (client.holding_registers(0x0002_addr, 2) = {0x0000, 0xff00}).message() << '\n';
  std::this_thread::sleep_for(1500ms);
  std::cout << (client.holding_registers(0x0001_addr, 2) = {0x0000, 0x0000}).message() << '\n';
  std::cout << (client.holding_registers(0x0002_addr, 2) = {0x0000, 0x0000}).message() << '\n';
}
catch (std::exception const & e)
{
  std::cout << e.what() << std::endl;
}