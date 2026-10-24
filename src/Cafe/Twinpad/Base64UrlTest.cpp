#include "Common/precompiled.h"
#include "Cafe/Twinpad/TwinpadClient.h"
#include "Cafe/HW/Latte/Renderer/Vulkan/VulkanRenderer.h"

#include <initializer_list>

namespace
{
std::string Base64UrlReference(const uint8_t* data, size_t size)
{
	static constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
	std::string encoded;
	for (size_t index = 0; index < size; index += 3)
	{
		const uint32_t block = (static_cast<uint32_t>(data[index]) << 16)
			| (index + 1 < size ? static_cast<uint32_t>(data[index + 1]) << 8 : 0)
			| (index + 2 < size ? static_cast<uint32_t>(data[index + 2]) : 0);
		encoded.push_back(alphabet[(block >> 18) & 63]);
		encoded.push_back(alphabet[(block >> 12) & 63]);
		if (index + 1 < size)
			encoded.push_back(alphabet[(block >> 6) & 63]);
		if (index + 2 < size)
			encoded.push_back(alphabet[block & 63]);
	}
	return encoded;
}
}

void VulkanRenderer_twinpad_readback_test()
{
	// TEST-075 — verifies SPEC-003: completed Twinpad readbacks are chosen newest-first without waiting.
	using Slot = VulkanRenderer::TwinpadReadbackSlot;
	std::array<Slot, VulkanRenderer::TWINPAD_READBACK_SLOT_COUNT> slots{};
	slots[0].pending = true;
	slots[0].sequence = 8;
	slots[0].commandBufferId = 10;
	slots[1].pending = true;
	slots[1].sequence = 9;
	slots[1].commandBufferId = 12;
	slots[2].pending = true;
	slots[2].sequence = 7;
	slots[2].commandBufferId = 9;
	cemu_assert_debug(VulkanRenderer::FindNewestCompletedTwinpadReadbackSlot(slots, 11) == 0);
	cemu_assert_debug(VulkanRenderer::FindNewestCompletedTwinpadReadbackSlot(slots, 13) == 1);
	cemu_assert_debug(VulkanRenderer::FindNewestCompletedTwinpadReadbackSlot(slots, 9) == slots.size());
	slots[1].pending = false;
	slots[2].pending = false;
	cemu_assert_debug(VulkanRenderer::FindNewestCompletedTwinpadReadbackSlot(slots, 13) == 0);

	// A partially allocated ring may have no completed slot while every
	// allocated slot is pending; callers must compare against slots.size().
	slots[0].pending = true;
	slots[0].commandBufferId = 13;
	slots[1].pending = true;
	slots[1].commandBufferId = 14;
	const size_t noCompletedSlot = VulkanRenderer::FindOldestCompletedTwinpadReadbackSlot(slots, 13);
	cemu_assert_debug(noCompletedSlot == slots.size());
	cemu_assert_debug(noCompletedSlot != 2);
}

void TwinpadOrientationMapper_test()
{
	// TEST-077 — verifies SPEC-004: absolute yaw/pitch/roll maps to the Cemu VPAD axis order.
	TwinpadInputState input{};
	input.yaw = 8192;
	input.pitch = -4096;
	input.roll = 16384;
	const auto mapped = TwinpadOrientationMapper::ToCemuTurns(input);
	cemu_assert_debug(std::abs(mapped[0] + 0.0625f) < 0.000001f);
	cemu_assert_debug(std::abs(mapped[1] - 0.125f) < 0.000001f);
	cemu_assert_debug(std::abs(mapped[2] - 0.25f) < 0.000001f);

	input.yaw = std::numeric_limits<int16_t>::min();
	input.pitch = std::numeric_limits<int16_t>::min();
	input.roll = std::numeric_limits<int16_t>::min();
	const auto extremes = TwinpadOrientationMapper::ToCemuTurns(input);
	cemu_assert_debug(extremes[0] == -0.5f);
	cemu_assert_debug(extremes[1] == -0.5f);
	cemu_assert_debug(extremes[2] == -0.5f);

	input = TwinpadInputState{};
	input.yaw = 8192;
	input.pitch = -4096;
	input.roll = -262;
	const auto halfSensitivity = TwinpadOrientationMapper::ToCemuTurns(input);
	cemu_assert_debug(std::abs(halfSensitivity[0] + 0.0625f) < 0.000001f);
	cemu_assert_debug(std::abs(halfSensitivity[1] - 0.125f) < 0.000001f);
	cemu_assert_debug(std::abs(halfSensitivity[2] + (262.0f / 32768.0f * 0.5f)) < 0.000001f);

	const std::array<float, 3> previousTurns{};
	const std::array<float, 3> currentTurns{0.125f, -0.125f, 0.25f};
	const auto gyroChange = TwinpadOrientationMapper::ToCemuGyroChange(previousTurns, currentTurns, 0.05f);
	cemu_assert_debug(std::abs(gyroChange[0] + 2.5f) < 0.000001f);
	cemu_assert_debug(std::abs(gyroChange[1] + 2.5f) < 0.000001f);
	cemu_assert_debug(std::abs(gyroChange[2] - 5.0f) < 0.000001f);

	const std::array<float, 3> beforeWrap{0.49f, -0.49f, 0.0f};
	const std::array<float, 3> afterWrap{-0.49f, 0.49f, 0.0f};
	const auto wrappedChange = TwinpadOrientationMapper::ToCemuGyroChange(beforeWrap, afterWrap, 0.05f);
	cemu_assert_debug(std::abs(wrappedChange[0] + 0.4f) < 0.000001f);
	cemu_assert_debug(std::abs(wrappedChange[1] + 0.4f) < 0.000001f);

	const auto tooSoon = TwinpadOrientationMapper::ToCemuGyroChange(previousTurns, currentTurns, 0.0005f);
	const auto stalled = TwinpadOrientationMapper::ToCemuGyroChange(previousTurns, currentTurns, 0.25f);
	const auto invalidInterval = TwinpadOrientationMapper::ToCemuGyroChange(
		previousTurns, currentTurns, std::numeric_limits<float>::quiet_NaN());
	cemu_assert_debug(tooSoon == std::array<float, 3>{});
	cemu_assert_debug(stalled == std::array<float, 3>{});
	cemu_assert_debug(invalidInterval == std::array<float, 3>{});
}

void TwinpadClient_base64url_test()
{
	const auto encode = [](std::initializer_list<uint8_t> bytes) {
		return TwinpadClient::Base64Url(bytes.begin(), bytes.size());
	};
	cemu_assert_debug(encode({}) == "");
	cemu_assert_debug(encode({0x66}) == "Zg");
	cemu_assert_debug(encode({0x66, 0x6f}) == "Zm8");
	cemu_assert_debug(encode({0x66, 0x6f, 0x6f}) == "Zm9v");
	cemu_assert_debug(encode({0xfb, 0xff}) == "-_8");

	std::vector<uint8_t> frame(1280 * 720 * 3);
	for (size_t index = 0; index < frame.size(); ++index)
		frame[index] = static_cast<uint8_t>((index * 37 + index / 19) % 256);
	const auto encoded = TwinpadClient::Base64Url(frame.data(), frame.size());
	cemu_assert_debug(encoded == Base64UrlReference(frame.data(), frame.size()));
	cemu_assert_debug(encoded.size() == (frame.size() * 4 + 2) / 3);
	cemu_assert_debug(encoded.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_") == std::string::npos);
}

namespace
{
struct H264ParameterSets
{
	bool sps{};
	bool pps{};
	bool idr{};
	bool constrainedBaselineLevel42{};
};

H264ParameterSets InspectAnnexB(const std::vector<uint8_t>& bytes)
{
	H264ParameterSets result;
	auto findStartCode = [&bytes](size_t from, size_t& prefixSize) {
		for (size_t index = from; index + 3 <= bytes.size(); ++index)
		{
			if (bytes[index] != 0 || bytes[index + 1] != 0)
				continue;
			if (bytes[index + 2] == 1)
			{
				prefixSize = 3;
				return index;
			}
			if (index + 4 <= bytes.size() && bytes[index + 2] == 0 && bytes[index + 3] == 1)
			{
				prefixSize = 4;
				return index;
			}
		}
		return bytes.size();
	};

	size_t prefixSize = 0;
	size_t start = findStartCode(0, prefixSize);
	while (start < bytes.size())
	{
		const size_t nal = start + prefixSize;
		size_t nextPrefixSize = 0;
		const size_t next = findStartCode(nal + 1, nextPrefixSize);
		if (nal < bytes.size())
		{
			const uint8_t type = bytes[nal] & 0x1f;
			result.sps |= type == 7;
			result.pps |= type == 8;
			result.idr |= type == 5;
			if (type == 7 && nal + 3 < bytes.size())
			{
				const uint8_t profileIdc = bytes[nal + 1];
				const uint8_t constraints = bytes[nal + 2];
				const uint8_t levelIdc = bytes[nal + 3];
				result.constrainedBaselineLevel42 |= profileIdc == 66 && (constraints & 0x40) != 0 && levelIdc == 42;
			}
		}
		start = next;
		prefixSize = nextPrefixSize;
	}
	return result;
}
}

void TwinpadClient_h264_encoder_test()
{
	// TEST-073 — verifies SPEC-003/006: real OpenH264 access units carry Annex B SPS/PPS and IDR metadata.
	auto& client = TwinpadClient::Instance();
	client.SetHardwareAccelerationEnabled(false);
	client.ResetVideoEncoder();
	cemu_assert_debug(!client.IsHardwareAccelerationEnabled());
	client.SetCaptureFps(30);
	std::vector<uint8_t> frame(64 * 64 * 3);
	for (size_t index = 0; index < frame.size(); ++index)
		frame[index] = static_cast<uint8_t>((index * 37 + index / 19) % 256);

	TwinpadEncodedFrame first;
	const bool firstEncoded = client.EncodeGamePadFrame(frame, 64, 64, first);
	cemu_assert_debug(firstEncoded);
	if (firstEncoded)
	{
		const auto nals = InspectAnnexB(first.annexB);
		cemu_assert_debug(first.idr);
		cemu_assert_debug(nals.idr);
		cemu_assert_debug(nals.sps);
		cemu_assert_debug(nals.pps);
		cemu_assert_debug(nals.constrainedBaselineLevel42);
	}
	cemu_assert_debug(!client.IsHardwareEncoderActive());
	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	TwinpadEncodedFrame second;
	const bool secondEncoded = client.EncodeGamePadFrame(frame, 64, 64, second);
	cemu_assert_debug(secondEncoded);
	if (secondEncoded)
		cemu_assert_debug(second.timestampUs > first.timestampUs);

	client.ResetVideoEncoder();
	TwinpadEncodedFrame afterReset;
	const bool resetEncoded = client.EncodeGamePadFrame(frame, 64, 64, afterReset);
	cemu_assert_debug(resetEncoded);
	if (resetEncoded)
	{
		const auto nals = InspectAnnexB(afterReset.annexB);
		cemu_assert_debug(afterReset.idr);
		cemu_assert_debug(nals.idr);
		cemu_assert_debug(nals.sps);
		cemu_assert_debug(nals.pps);
		cemu_assert_debug(nals.constrainedBaselineLevel42);
	}

	client.ResetVideoEncoder();
	client.SetHardwareAccelerationEnabled(false);
}

void TwinpadClient_hardware_encoder_test()
{
	// TEST-082 — verifies CR-020: hardware encoding is opt-in and uses the same H.264 contract when available.
	auto& client = TwinpadClient::Instance();
	client.SetHardwareAccelerationEnabled(false);
	client.ResetVideoEncoder();
	cemu_assert_debug(!client.IsHardwareAccelerationEnabled());

	std::vector<uint8_t> frame(64 * 64 * 3, 128);
	TwinpadEncodedFrame software;
	const bool softwareEncoded = client.EncodeGamePadFrame(frame, 64, 64, software);
	cemu_assert_debug(softwareEncoded);
	if (!softwareEncoded)
	{
		client.ResetVideoEncoder();
		return;
	}
	cemu_assert_debug(!client.IsHardwareEncoderActive());

	client.SetHardwareAccelerationEnabled(true);
	cemu_assert_debug(client.IsHardwareAccelerationEnabled());
	TwinpadEncodedFrame accelerated;
	const bool acceleratedEncoded = client.EncodeGamePadFrame(frame, 64, 64, accelerated);
	cemu_assert_debug(acceleratedEncoded);
	if (acceleratedEncoded)
	{
		const auto nals = InspectAnnexB(accelerated.annexB);
		cemu_assert_debug(accelerated.idr);
		cemu_assert_debug(nals.idr);
		cemu_assert_debug(nals.sps);
		cemu_assert_debug(nals.pps);
		cemu_assert_debug(nals.constrainedBaselineLevel42);
	}

	client.SetHardwareAccelerationEnabled(false);
	TwinpadEncodedFrame disabled;
	const bool disabledEncoded = client.EncodeGamePadFrame(frame, 64, 64, disabled);
	cemu_assert_debug(disabledEncoded);
	cemu_assert_debug(!client.IsHardwareEncoderActive());
	client.ResetVideoEncoder();
}
