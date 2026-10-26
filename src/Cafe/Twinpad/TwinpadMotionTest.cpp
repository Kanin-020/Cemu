#include "Common/precompiled.h"
#include "Cafe/Twinpad/TwinpadClient.h"

void TwinpadMotionMapper_test()
{
	// TEST-084 — verifies SPEC-004/CR-021: Android inertial samples follow the reference DSU mapping.
	TwinpadInputState input{};
	input.motionValid = true;
	input.accel = {1.0f, -2.0f, 3.0f};
	input.gyro = {1.0f, -2.0f, 3.0f};

	const auto gyro = TwinpadMotionMapper::ToCemuGyroRadiansPerSecond(input);
	const auto accel = TwinpadMotionMapper::ToCemuAcceleration(input);
	cemu_assert_debug(std::abs(gyro[0] - 0.9f) < 0.000001f);
	cemu_assert_debug(std::abs(gyro[1] + 1.8f) < 0.000001f);
	cemu_assert_debug(std::abs(gyro[2] - 2.7f) < 0.000001f);
	cemu_assert_debug(std::abs(accel[0] - 1.0f * 9.8066f / 100.0f) < 0.000001f);
	cemu_assert_debug(std::abs(accel[1] + 2.0f * 9.8066f / 100.0f) < 0.000001f);
	cemu_assert_debug(std::abs(accel[2] - 3.0f * 9.8066f / 100.0f) < 0.000001f);

	cemu_assert_debug(std::abs(TwinpadMotionMapper::ElapsedSeconds(1'000'000, 1'020'000) - 0.02f) < 0.000001f);
	cemu_assert_debug(TwinpadMotionMapper::ElapsedSeconds(10, 10) == 0.0f);
	cemu_assert_debug(TwinpadMotionMapper::ElapsedSeconds(20, 10) == 0.0f);
}
