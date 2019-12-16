#include "roboteq/driver.hpp"

namespace boarai::hardware::roboteq
{
  driver::driver()
  {
    // TODO (smiracco): define proper constructor
  }

  auto driver::emergency_stop()
  {
    return client.holding_registers(emergency_stop, 2) = {0x0000, 0x0000};
  }

}  // namespace boarai::hardware::roboteq
