#include "Common/precompiled.h"
#include "Cafe/Twinpad/TwinpadClient.h"

void TwinpadGyroMonitor_test()
{
	// TEST-079 — verifies SPEC-006: the Twinpad gyro monitor is opt-in and maps received axes.
	auto& client = TwinpadClient::Instance();
	client.SetGyroMonitorEnabled(false);
	{
		std::scoped_lock lock(client.m_inputMutex);
		client.m_gyroMonitorState = TwinpadInputState{};
		client.m_fusedMotionState = TwinpadFusedMotionState{};
		client.m_hasGyroMonitorState = false;
	}
	cemu_assert_debug(!client.IsGyroMonitorEnabled());
	cemu_assert_debug(!client.GetGyroMonitorState().motionValid);
	client.SetGyroMonitorEnabled(true);
	cemu_assert_debug(client.IsGyroMonitorEnabled());
	if (!client.IsGyroMonitorEnabled())
		return;

	TwinpadInputState input{};
	input.motionValid = true;
	input.motionTimestampUs = 1'000'000;
	input.accel = {0.0f, 9.8066f, 0.0f};
	input.gyro = {1.0f, -2.0f, 3.0f};
	{
		std::scoped_lock lock(client.m_inputMutex);
		client.m_gyroMonitorState = input;
		client.m_hasGyroMonitorState = true;
		client.m_lastMotionReceived = std::chrono::steady_clock::now();
		client.m_hasFreshMotionInput = true;
	}
	const auto received = client.GetGyroMonitorState();
	cemu_assert_debug(received.motionValid);
	cemu_assert_debug(received.motionTimestampUs == 1'000'000);
	cemu_assert_debug(received.accel == std::array<float, 3>{0.0f, 9.8066f, 0.0f});
	cemu_assert_debug(received.gyro == std::array<float, 3>{1.0f, -2.0f, 3.0f});

	const auto mappedGyro = TwinpadMotionMapper::ToCemuGyroRadiansPerSecond(received);
	const auto mappedAccel = TwinpadMotionMapper::ToCemuAcceleration(received);
	cemu_assert_debug(mappedGyro == std::array<float, 3>{0.9f, -2.7f, -1.8f});
	cemu_assert_debug(std::abs(mappedAccel[0]) < 0.000001f);
	cemu_assert_debug(std::abs(mappedAccel[1]) < 0.000001f);
	cemu_assert_debug(std::abs(mappedAccel[2] + 9.8066f * 9.8066f / 100.0f) < 0.0001f);

	TwinpadFusedMotionState fused{};
	fused.valid = true;
	fused.gyroOrientation = {0.125f, -0.25f, 0.5f};
	client.SetFusedMotionState(fused);
	const auto observedFusion = client.GetFusedMotionState();
	cemu_assert_debug(observedFusion.valid);
	cemu_assert_debug(observedFusion.gyroOrientation == fused.gyroOrientation);
	client.SetGyroMonitorEnabled(false);
	{
		std::scoped_lock lock(client.m_inputMutex);
		client.m_gyroMonitorState = TwinpadInputState{};
		client.m_fusedMotionState = TwinpadFusedMotionState{};
		client.m_hasGyroMonitorState = false;
		client.m_hasFreshMotionInput = false;
	}
}
