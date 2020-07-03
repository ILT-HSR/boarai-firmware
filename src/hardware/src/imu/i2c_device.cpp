#include "imu/i2c_device.hpp"

extern "C"
{
#include "imu/smbus.h"
}

#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <cerrno>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <linux/i2c-dev.h>
#include <system_error>
#include <variant>
#include <vector>

namespace boarai::hardware
{

  auto constexpr word_size = sizeof(std::uint16_t);

  i2c_device::i2c_device(std::filesystem::path bus, std::uint8_t address)
      : m_bus_handle{open(bus.c_str(), O_RDWR)}
  {
    if (m_bus_handle == -1)
    {
      throw std::system_error{errno, std::system_category()};
    }

    if (ioctl(m_bus_handle, I2C_SLAVE, address) == -1)
    {
      throw std::system_error{errno, std::system_category()};
    }
  }

  auto i2c_device::write(std::uint8_t address, std::byte data) -> std::error_code
  {
    return {-i2c_smbus_write_byte_data(m_bus_handle, address, static_cast<std::uint8_t>(data)), std::system_category()};
  }

  auto i2c_device::write(std::uint8_t address, std::vector<std::byte> data) -> std::error_code
  {
    for (auto write_number = 0ull; write_number < data.size() / word_size; ++write_number, address += word_size)
    {
      auto word = std::uint16_t{};
      std::memcpy(&word, data.data() + write_number * word_size, sizeof(word));
      auto error = i2c_smbus_write_word_data(m_bus_handle, address, word);
      if (error < 0)
      {
        return {-error, std::system_category()};
      }
    }

    return (data.size() % word_size) ? write(address, data.back()) : std::error_code{};
  }

  auto i2c_device::read(std::uint8_t address, std::size_t amount) -> std::variant<std::error_code, std::vector<std::byte>>
  {
    auto data = std::vector<std::byte>(amount);

    for (auto read_number = 0ull; read_number < amount / sizeof(std::uint16_t); ++read_number, address += word_size)
    {
      auto received = i2c_smbus_read_word_data(m_bus_handle, address);
      if (received < 0)
      {
        return std::error_code{-received, std::system_category()};
      }
      std::memcpy(data.data() + read_number * word_size, &received, word_size);
    }

    if (amount % word_size)
    {
      auto received = i2c_smbus_read_byte_data(m_bus_handle, address);
      if (received < 0)
      {
        return std::error_code{-received, std::system_category()};
      }
      data.back() = static_cast<std::byte>(received);
    }

    return data;
  }

}  // namespace boarai::hardware