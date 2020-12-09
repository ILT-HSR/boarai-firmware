#include "support/string_utility.hpp"
#include "tank_drive_debug/tank_drive_debug.hpp"

namespace boarai::hardware
{

  auto tank_drive_debug::start_publishers() -> void
  {
    m_drive_velocity_publisher = create_publisher<topic::drive_velocity_t>(topic::drive_velocity, default_topic_policy);
  }

}  // namespace boarai::hardware