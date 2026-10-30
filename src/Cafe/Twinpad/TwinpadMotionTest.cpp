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

void TwinpadMotionQueue_test()
{
	// TEST-111 — verifies CR-045: pending motion samples drain oldest→newest, in
	// order, and a second drain leaves the queue empty (no duplicated samples).
	auto& motionClient = TwinpadClient::Instance();
	{
		std::scoped_lock lock(motionClient.m_inputMutex);
		motionClient.m_pendingMotionSamples.clear();
		for (uint32_t seq = 10; seq <= 30; seq += 10)
		{
			TwinpadMotionSample pending{};
			pending.timestampUs = 1'000'000ull + seq;
			pending.seq = seq;
			pending.accel = {static_cast<float>(seq), 0.0f, 0.0f};
			pending.gyro = {0.0f, static_cast<float>(seq), 0.0f};
			motionClient.m_pendingMotionSamples.push_back(pending);
		}
	}
	const auto drained = motionClient.DrainPendingMotionSamples();
	cemu_assert_debug(drained.size() == 3);
	cemu_assert_debug(drained[0].seq == 10 && drained[1].seq == 20 && drained[2].seq == 30);
	cemu_assert_debug(drained[0].timestampUs == 1'000'010ull && drained[2].timestampUs == 1'000'030ull);
	cemu_assert_debug(drained[0].accel == std::array<float, 3>{10.0f, 0.0f, 0.0f});
	cemu_assert_debug(drained[1].gyro == std::array<float, 3>{0.0f, 20.0f, 0.0f});
	cemu_assert_debug(motionClient.DrainPendingMotionSamples().empty());

	// TEST-112 — verifies CR-045: the queue is bounded with drop-old; the newest
	// kMotionQueueMaxSamples samples survive.
	{
		std::scoped_lock lock(motionClient.m_inputMutex);
		motionClient.m_pendingMotionSamples.clear();
		for (uint32_t seq = 1; seq <= TwinpadClient::kMotionQueueMaxSamples + 5; ++seq)
		{
			TwinpadMotionSample pending{};
			pending.timestampUs = 2'000'000ull + seq;
			pending.seq = seq;
			motionClient.m_pendingMotionSamples.push_back(pending);
		}
	}
	const auto capped = motionClient.DrainPendingMotionSamples();
	cemu_assert_debug(capped.size() == TwinpadClient::kMotionQueueMaxSamples);
	cemu_assert_debug(capped.front().seq == 6);
	cemu_assert_debug(capped.back().seq == TwinpadClient::kMotionQueueMaxSamples + 5);
	cemu_assert_debug(motionClient.DrainPendingMotionSamples().empty());
}
