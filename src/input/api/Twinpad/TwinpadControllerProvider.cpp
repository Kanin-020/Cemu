#include "input/api/Twinpad/TwinpadControllerProvider.h"
#include "input/api/Twinpad/TwinpadController.h"
#include "Cafe/Twinpad/Client/TwinpadClient.h"

std::vector<std::shared_ptr<ControllerBase>> TwinpadControllerProvider::get_controllers()
{
	// The phone pad is a single device. It is always listed so it can be selected in the
	// input settings; raw_state() returns an empty state while the client is not linked.
	std::vector<std::shared_ptr<ControllerBase>> result;
	result.emplace_back(std::make_shared<TwinpadController>(0));
	return result;
}
