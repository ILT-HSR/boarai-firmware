#include "imu/imu.hpp"

extern "C"
{
#include "imu/smbus.h"
}

#include "rclcpp_components/register_node_macro.hpp"
#include "support/interfaces.hpp"

#include <fcntl.h>
#include <stdio.h>
#include <sys/ioctl.h>
#include <sys/stat.h>

#include <linux/i2c-dev.h>

auto constexpr node_name{"imu"};

namespace boarai::hardware
{

  imu::imu(rclcpp::NodeOptions const & options)
      : fmt_node{node_name, ros_namespace, options}
  {
    auto bus_handle = open("/dev/i2c-0", O_RDWR);
    if (ioctl(bus_handle, I2C_SLAVE, 0x28) < 0)
    {
      log_error("Failed to set slave ID! reason: {}", sys_errlist[errno]);
      return;
    }

    if (i2c_smbus_write_byte(bus_handle, 0x00) < 0)
    {
      log_warning("Failed to set address for read! reason: {}", sys_errlist[errno]);
    }

    auto id = i2c_smbus_read_byte(bus_handle);
    if (id < 0)
    {
      log_error("Failed to read chip id! reason: {}", sys_errlist[errno]);
    }
    else
    {
      log_info("Read chip id {}", id);
    }
  }

}  // namespace boarai::hardware

RCLCPP_COMPONENTS_REGISTER_NODE(boarai::hardware::imu)