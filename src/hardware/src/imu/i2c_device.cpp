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
    i2c_smbus_write_byte(m_bus_handle, address);
    return {-i2c_smbus_write_byte(m_bus_handle, static_cast<std::uint8_t>(data)), std::system_category()};
  }

  auto i2c_device::write(std::uint8_t address, std::vector<std::byte> data) -> std::error_code
  {
    i2c_smbus_write_byte(m_bus_handle, address);
    return {-i2c_smbus_write_i2c_block_data(m_bus_handle, address, data.size(), reinterpret_cast<std::uint8_t *>(data.data())),
            std::system_category()};
  }

  auto i2c_device::read(std::uint8_t address, std::size_t amount) -> std::variant<std::error_code, std::vector<std::byte>>
  {
    auto data = std::vector<std::byte>{};

    for (auto i = 0ull; i < amount; ++i)
    {
      auto bus_data = i2c_smbus_read_byte_data(m_bus_handle, address++);
      if (bus_data < 0)
      {
        return std::error_code{errno, std::system_category()};
      }
      data.push_back(static_cast<std::byte>(bus_data));
    }
    return data;

    // auto out = data.data();

    // while (amount)
    // {
    //   auto bus_data = i2c_smbus_read_word_data(m_bus_handle, address);
    //   if (bus_data < 0)
    //   {
    //     return std::error_code{errno, std::system_category()};
    //   }
    //   std::memcpy(out, &bus_data, 2);
    //   address += 2;
    //   out += 2;
    //   amount -= 2;
    // }

    // return data;
  }

}  // namespace boarai::hardware