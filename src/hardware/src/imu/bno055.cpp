#include "imu/bno055.hpp"

#include "support/fallible.hpp"

#include <fmt/format.h>

#include <chrono>
#include <cstring>
#include <filesystem>
#include <new>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>

namespace boarai::hardware
{
  auto constexpr default_chip_id = std::byte{0xa0};
  auto constexpr reset = std::byte{0x20};

  auto constexpr switch_to_configuration_mode_duration = std::chrono::milliseconds{19};
  auto constexpr switch_from_configuration_mode_duration = std::chrono::milliseconds{7};
  auto constexpr wait_after_reset_duration = std::chrono::milliseconds{650};

  enum struct device_register : std::uint8_t
  {
    chip_id = 0x00,
    accelerometer_id = 0x01,
    magnetometer_id = 0x02,
    gyroscope_id = 0x03,
    // ...
    euler_heading_lsb = 0x1a,
    // ...
    operation_mode = 0x3d,
    power_mode = 0x3e,
    trigger_system_operation = 0x3f,
  };

  enum struct operation_mode : std::uint8_t
  {
    configuration = 0,
    accelerometer_only = 1,
    magnetometer_only = 2,
    gyroscopye_only = 3,
    accelerometer_plus_magnetometer = 4,
    accelerometer_plus_gyroscrope = 5,
    magnetometer_plus_gyroscrope = 6,
    accelerometer_plus_magnetometer_plus_gyroscope = 7,
    inertial_measurement_unit = 8,
    compass = 9,
    magnet_for_gyroscope = 10,
    ndof_without_fast_calibration = 11,
    ndof_with_fast_calibration = 12,
  };

  enum struct power_mode : std::uint8_t
  {
    normal = 0,
    low = 1,
    suspend = 2,
  };

  namespace
  {
    auto throw_on_error(std::error_code error, std::string message) -> void
    {
      if (error)
      {
        throw std::system_error{error, message};
      }
    }
  }  // namespace

  bno055::bno055(std::filesystem::path bus)
      : m_device{bus, 0x28}
  {
    if (expect(read(device_register::chip_id)) != default_chip_id)
    {
      throw std::runtime_error{"The device does not seem to be a BNO055."};
    }

    throw_on_error(write(device_register::trigger_system_operation, reset), "Failed to reset the device");

    std::this_thread::sleep_for(wait_after_reset_duration);

    enter(operation_mode::configuration);
    enter(power_mode::normal);

    throw_on_error(write(device_register::trigger_system_operation, std::byte{0}), "Failed to enter normal power mode!");
    throw_on_error(enter(operation_mode::ndof_with_fast_calibration), "Failed to enter NDOF mode");
  }

  auto bno055::euler_orientation() -> orientation
  {
    auto raw_data = expect(read(device_register::euler_heading_lsb, 6));
    auto converted = orientation{};
    std::memcpy(&converted, raw_data.data(), raw_data.size());
    return converted;
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

  auto bno055::write(device_register reg, std::byte data) -> std::error_code
  {
    return m_device.write(static_cast<std::underlying_type_t<device_register>>(reg), data);
  }

  auto bno055::write(device_register reg, std::vector<std::byte> data) -> std::error_code
  {
    return m_device.write(static_cast<std::underlying_type_t<device_register>>(reg), data);
  }

  auto bno055::enter(operation_mode mode) -> std::error_code
  {
    auto switch_error = write(device_register::operation_mode, static_cast<std::byte>(mode));
    if (switch_error)
    {
      return switch_error;
    }
    std::this_thread::sleep_for(mode == operation_mode::configuration ? switch_to_configuration_mode_duration
                                                                      : switch_from_configuration_mode_duration);
    return {};
  }

  auto bno055::enter(power_mode mode) -> std::error_code
  {
    auto switch_error = write(device_register::power_mode, static_cast<std::byte>(mode));
    if (switch_error)
    {
      return switch_error;
    }
    return {};
  }

}  // namespace boarai::hardware