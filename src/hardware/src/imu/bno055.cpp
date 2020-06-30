#include "imu/bno055.hpp"

#include "support/fallible.hpp"

#include <filesystem>
#include <new>
#include <stdexcept>
#include <type_traits>

namespace boarai::hardware
{
  auto constexpr default_chip_id = std::byte{0xa0};

  enum struct device_register : std::uint8_t
  {
    chip_id = 0x00,
    accelerometer_id = 0x01,
    magnetometer_id = 0x02,
    gyroscope_id = 0x03,
  };

  bno055::bno055(std::filesystem::path bus)
      : m_device{bus, 0x28}
  {
    if (expect(read(device_register::chip_id)) != default_chip_id)
    {
      throw std::runtime_error{"The device does not seem to be a BNO055."};
    }
  }

  auto bno055::read(device_register reg) -> fallible<std::byte>
  {
    auto result = read(reg, 1);
    if (auto data = expect(std::nothrow, result))
    {
      return data->front();
    }
    return get_error(result);
  }

  auto bno055::read(device_register reg, std::size_t amount) -> fallible<std::vector<std::byte>>
  {
    return m_device.read(static_cast<std::underlying_type_t<device_register>>(reg), amount);
  }

}  // namespace boarai::hardware