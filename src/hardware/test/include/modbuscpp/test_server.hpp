#ifndef BOARAI_HARDWARE_TEST_MODBUSCPP_TEST_SERVER_HPP
#define BOARAI_HARDWARE_TEST_MODBUSCPP_TEST_SERVER_HPP

#include "modbuscpp/address.hpp"
#include "modbuscpp/context.hpp"

#include <modbus/modbus.h>

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>

namespace boarai::hardware::test
{
  namespace detail
  {
    struct modbus_mapping_t_delete
    {
      auto operator()(modbus_mapping_t * mapping) const noexcept -> void;
    };
  }  // namespace detail

  struct test_server
  {
    struct mapping_parameters
    {
      int coils;
      int discrete_inputs;
      int holding_registers;
      int input_registers;
    };

    using mapping_pointer = std::unique_ptr<modbus_mapping_t, detail::modbus_mapping_t_delete>;

    test_server(mapping_parameters mapping_parameters, std::uint16_t port);

    ~test_server();

    auto run_once() -> void;

    auto coil(modbus::address address) const -> bool;

    auto coil(modbus::address address, bool value) -> void;

  private:
    modbus::context m_context;
    mapping_pointer m_data{};
    std::mutex mutable m_data_mutex{};
    int m_socket;
  };
}  // namespace boarai::hardware::test

#endif