#ifndef BOARAI_HARDWARE_LAYER_CONSTANTS_HPP
#define BOARAI_HARDWARE_LAYER_CONSTANTS_HPP

namespace boarai::hardware
{

  auto constexpr LAYER_NAMESPACE = "/boarai/hardware";

  auto constexpr HARDWARE_SERVICE_SET_DRIVE_VELOCITY = "set_drive_velocity";
  auto constexpr HARDWARE_SERVICE_GET_MAXIMUM_ANGULAR_VELOCITY = "get_maximum_angular_velocity";
  auto constexpr HARDWARE_TOPIC_GLOBAL_POSITION = "global_position";
  auto constexpr HARDWARE_TOPIC_BATTERY_VOLTAGE = "battery_voltage";

}  // namespace boarai::hardware

#endif