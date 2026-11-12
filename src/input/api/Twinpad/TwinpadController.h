#pragma once

#include "input/api/Controller.h"
#include "input/api/Twinpad/TwinpadControllerProvider.h"

class TwinpadController : public Controller<TwinpadControllerProvider>
{
public:
	TwinpadController(uint32 index);

	std::string_view api_name() const override
	{
		return to_string(InputAPI::Twinpad);
	}
	InputAPI::Type api() const override { return InputAPI::Twinpad; }

	bool connect() override;
	bool is_connected() override;

protected:
	ControllerState raw_state() override;

private:
	uint32 m_index;
};
