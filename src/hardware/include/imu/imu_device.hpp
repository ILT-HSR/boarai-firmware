#ifndef BOARAI_HARDWARE_IMU_DEVICE_HPP
#define BOARAI_HARDWARE_IMU_DEVICE_HPP

#include <cstdint>

namespace boarai::hardware
{

  struct imu_device
  {
    struct orientation
    {
      std::int16_t heading;
      std::int16_t roll;
      std::int16_t pitch;
    };

    auto virtual euler_orientation() -> orientation = 0;
  };

}  // namespace boarai::hardware

#endif