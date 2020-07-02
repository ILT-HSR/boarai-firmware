#include "imu/imu.hpp"

namespace boarai::hardware
{

  auto imu::start_publishers() -> void
  {
    m_orientation_publisher = create_publisher<topic::imu_orientation_t>(topic::imu_orientation, 10);
  }

}  // namespace boarai::hardware