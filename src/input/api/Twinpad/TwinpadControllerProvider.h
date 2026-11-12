#pragma once

#include "input/api/ControllerProvider.h"

// Exposes the Twinpad phone gamepad (TwinpadClient) as a regular Cemu input
// device so it can be selected in Input settings → Controller 1.
class TwinpadControllerProvider : public ControllerProviderBase
{
public:
	TwinpadControllerProvider() = default;

	inline static InputAPI::Type kAPIType = InputAPI::Twinpad;
	InputAPI::Type api() const override { return kAPIType; }

	std::vector<std::shared_ptr<ControllerBase>> get_controllers() override;
};
