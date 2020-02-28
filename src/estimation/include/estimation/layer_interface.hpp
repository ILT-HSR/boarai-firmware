#ifndef BOARAI_ESTIMATION_LAYER_INTERFACE_HPP
#define BOARAI_ESTIMATION_LAYER_INTERFACE_HPP

#include "support/messages.hpp"

namespace boarai::estimation
{

  auto constexpr ros_namespace = "/boarai/estimation";

  namespace topic
  {
    using estimated_velocity_t = messages::PolarVelocity;
    auto estimated_velocity{"estimated_velocity"};
  }  // namespace topic

}  // namespace boarai::estimation

#endif