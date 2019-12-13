#include "roboteq/driver.hpp"
#include "roboteq/constants.hpp"

#include "modbuscpp/modbuscpp.hpp"

namespace boarai::hardware::roboteq
{
	driver::driver()
	{
		//TODO (smiracco): define proper constructor
	}
		
	auto driver::emergency_stop()
	{
		return client.holding_registers(emergency_stop, 2) = {0x0000, 0x0000};
	}
	
}
