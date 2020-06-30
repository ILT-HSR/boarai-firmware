#ifndef BOARAI_HARDWARE_BNO055_HPP
#define BOARAI_HARDWARE_BNO055_HPP

#include "imu/i2c_device.hpp"
#include "imu/imu_device.hpp"
#include "support/fallible.hpp"

#include <cstddef>
#include <filesystem>
#include <system_error>
#include <vector>

namespace boarai::hardware
{

  enum struct device_register : std::uint8_t;
  enum struct operation_mode : std::uint8_t;
  enum struct power_mode : std::uint8_t;

  struct bno055 : imu_device
  {
    explicit bno055(std::filesystem::path bus);

    auto euler_orientation() -> imu_device::orientation override;

  private:
    auto read(device_register reg) -> fallible<std::byte>;
    auto read(device_register reg, std::size_t amount) -> fallible<std::vector<std::byte>>;
    auto write(device_register reg, std::byte data) -> std::error_code;
    auto write(device_register reg, std::vector<std::byte> data) -> std::error_code;

    auto enter(operation_mode mode) -> std::error_code;
    auto enter(power_mode mode) -> std::error_code;

    i2c_device m_device;
  };

}  // namespace boarai::hardware

#endif