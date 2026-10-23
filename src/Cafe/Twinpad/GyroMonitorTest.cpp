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
		client.m_hasGyroMonitorState = false;
	}
	cemu_assert_debug(!client.IsGyroMonitorEnabled());
	cemu_assert_debug(!client.GetGyroMonitorState().gyroValid);
	client.SetGyroMonitorEnabled(true);
	cemu_assert_debug(client.IsGyroMonitorEnabled());
	if (!client.IsGyroMonitorEnabled())
		return;

	TwinpadInputState input{};
	input.gyroValid = true;
	input.yaw = 8192;
	input.pitch = -4096;
	input.roll = 16384;
	{
		std::scoped_lock lock(client.m_inputMutex);
		client.m_gyroMonitorState = input;
		client.m_hasGyroMonitorState = true;
	}
	const auto received = client.GetGyroMonitorState();
	const auto mapped = TwinpadOrientationMapper::ToCemuTurns(received);
	cemu_assert_debug(received.gyroValid);
	cemu_assert_debug(received.yaw == 8192 && received.pitch == -4096 && received.roll == 16384);
	cemu_assert_debug(std::abs(mapped[0] + 0.0625f) < 0.000001f);
	cemu_assert_debug(std::abs(mapped[1] - 0.125f) < 0.000001f);
	cemu_assert_debug(std::abs(mapped[2] - 0.25f) < 0.000001f);
	const auto monitorRadians = TwinpadOrientationMapper::ToMonitorRadians(received);
	cemu_assert_debug(std::abs(monitorRadians[2] - 0.5 * 6.283185307179586) < 0.000001);

	TwinpadInputState capturedRoll{};
	capturedRoll.gyroValid = true;
	capturedRoll.yaw = 11655;
	capturedRoll.pitch = -748;
	capturedRoll.roll = -262;
	const auto capturedAngles = TwinpadOrientationMapper::ToMonitorRadians(capturedRoll);
	const auto capturedEndPoint = TwinpadOrientationMapper::ToMonitorEndPoint(capturedRoll, 2, 1.0);
	cemu_assert_debug(std::abs(capturedAngles[2] - (-262.0 / 32768.0 * 6.283185307179586)) < 0.000001);
	cemu_assert_debug(TwinpadOrientationMapper::RollPointsRight(capturedRoll));
	cemu_assert_debug(capturedEndPoint[0] > 0.0 && std::abs(capturedEndPoint[0]) > std::abs(capturedEndPoint[1]));

	TwinpadInputState zeroRoll{};
	zeroRoll.gyroValid = true;
	const auto zeroRollEndPoint = TwinpadOrientationMapper::ToMonitorEndPoint(zeroRoll, 2, 1.0);
	cemu_assert_debug(TwinpadOrientationMapper::RollPointsRight(zeroRoll));
	cemu_assert_debug(zeroRollEndPoint[0] > 0.0 && std::abs(zeroRollEndPoint[0]) >= std::abs(zeroRollEndPoint[1]));
	client.SetGyroMonitorEnabled(false);
	{
		std::scoped_lock lock(client.m_inputMutex);
		client.m_gyroMonitorState = TwinpadInputState{};
		client.m_hasGyroMonitorState = false;
	}
}
