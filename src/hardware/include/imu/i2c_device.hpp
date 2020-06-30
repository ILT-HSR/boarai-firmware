#ifndef BOARAI_HARDWARE_I2C_DEVICE_HPP
#define BOARAI_HARDWARE_I2C_DEVICE_HPP

#include <cstddef>
#include <cstdint>
#include <filesystem>
#include <system_error>
#include <variant>
#include <vector>

namespace boarai::hardware
{

  struct i2c_device
  {
    i2c_device(std::filesystem::path bus, std::uint8_t address);

    auto write(std::uint8_t address, std::byte data) -> std::error_code;
    auto write(std::uint8_t address, std::vector<std::byte> data) -> std::error_code;
    auto read(std::uint8_t address, std::size_t amount) -> std::variant<std::error_code, std::vector<std::byte>>;

  private:
    int const m_bus_handle;
  };

}  // namespace boarai::hardware

#endif