#ifndef BOARAI_HARDWARE_BNO055_HPP
#define BOARAI_HARDWARE_BNO055_HPP

#include "imu/i2c_device.hpp"
#include "imu/imu_device.hpp"
#include "support/fallible.hpp"

#include <cstddef>
#include <filesystem>
#include <vector>

namespace boarai::hardware
{

  enum struct device_register : std::uint8_t;

  struct bno055 : imu_device
  {
    explicit bno055(std::filesystem::path bus);

    auto euler_orientation() -> imu_device::orientation override;

  private:
    auto read(device_register reg) -> fallible<std::byte>;
    auto read(device_register reg, std::size_t amount) -> fallible<std::vector<std::byte>>;

    i2c_device m_device;
  };

}  // namespace boarai::hardware

#endif