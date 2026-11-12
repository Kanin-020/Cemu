#include "input/api/Twinpad/TwinpadController.h"
#include "Cafe/Twinpad/Client/TwinpadClient.h"

TwinpadController::TwinpadController(uint32 index)
	: base_type(fmt::format("twinpad-{}", index), fmt::format("Twinpad {}", index + 1)), m_index(index)
{
}

bool TwinpadController::connect()
{
	return is_connected();
}

bool TwinpadController::is_connected()
{
	return TwinpadClient::Instance().IsConnected();
}

ControllerState TwinpadController::raw_state()
{
	ControllerState result{};
	if (!is_connected())
		return result;

	const TwinpadInputState input = TwinpadClient::Instance().GetInputState();

	// Same button order as VPADController::apply_twinpad_input (A,B,X,Y,L,R,
	// stick L, stick R, +, -) so the mapping matches the Wii U Gamepad.
	for (int i = 0; i < 10; ++i)
	{
		if (input.buttons & (1u << i))
			result.buttons.SetButtonState(i, true);
	}
	// D-pad: 0=up, 1=up-right, ... 7=up-left, 8=centered (same as VPADController::apply_twinpad_input)
	const uint8 dpad = input.dpad;
	if (dpad == 0 || dpad == 1 || dpad == 7) result.buttons.SetButtonState(kButtonUp, true);
	if (dpad >= 1 && dpad <= 3) result.buttons.SetButtonState(kButtonRight, true);
	if (dpad >= 3 && dpad <= 5) result.buttons.SetButtonState(kButtonDown, true);
	if (dpad >= 5 && dpad <= 7) result.buttons.SetButtonState(kButtonLeft, true);

	result.axis.x = static_cast<float>(input.stickLx) / 127.0f;
	result.axis.y = -static_cast<float>(input.stickLy) / 127.0f;
	result.rotation.x = static_cast<float>(input.stickRx) / 127.0f;
	result.rotation.y = -static_cast<float>(input.stickRy) / 127.0f;
	result.trigger.x = static_cast<float>(input.triggerL2) / 255.0f;
	result.trigger.y = static_cast<float>(input.triggerR2) / 255.0f;
	return result;
}
