#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <cmath>
#include <cstdint>
#include <deque>
#include <initializer_list>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

enum class TwinpadVideoMetricStage : uint8_t
{
	READBACK_COPY,
	READBACK_READY,
	PIXEL_CONVERT,
	I420_CONVERT,
	HARDWARE_ENCODER,
	OPENH264,
	ENCODE_PIPELINE,
	BASE64_JSON,
	IPC_SEND,
	COUNT,
};

enum class TwinpadVideoMetricEvent : uint8_t
{
	DRC_CALLBACK,
	CAPTURE_ACCEPTED,
	READBACK_QUEUED,
	READBACK_COMPLETED,
	READBACK_SLOT_BUSY,
	FRAME_QUEUED,
	FRAME_ENCODED,
	FRAME_SENT,
	COUNT,
};

struct TwinpadEncodedFrame
{
	std::vector<uint8_t> annexB;
	uint64_t timestampUs{};
	bool idr{};
};

struct TwinpadQr
{
	std::string uri;
	std::vector<std::string> modules;
	bool cemuLive{};
};

struct TwinpadInputState
{
	uint32_t seq{};
	int8_t stickLx{}, stickLy{}, stickRx{}, stickRy{};
	uint8_t dpad{8}, triggerL2{}, triggerR2{};
	uint16_t buttons{}, touchX{}, touchY{};
	bool touchPressed{};
	bool gyroValid{};
	int16_t yaw{}, pitch{}, roll{};
};

namespace TwinpadOrientationMapper
{
// Convert absolute Twinpad turns to the axis order used by Cemu's VPAD orientation.
inline std::array<float, 3> ToCemuTurns(const TwinpadInputState& input)
{
	constexpr float kInt16Turns = 32768.0f;
	constexpr float kCemuGyroSensitivity = 0.5f;
	return {
		static_cast<float>(input.pitch) / kInt16Turns * kCemuGyroSensitivity,
		static_cast<float>(input.yaw) / kInt16Turns * kCemuGyroSensitivity,
		static_cast<float>(input.roll) / kInt16Turns * kCemuGyroSensitivity,
	};
}

// Convert consecutive absolute orientation snapshots into VPAD angular velocity turns/second.
// The 0.5x sensitivity makes the signed orientation wrap every half turn.
inline std::array<float, 3> ToCemuGyroChange(
	const std::array<float, 3>& previousTurns,
	const std::array<float, 3>& currentTurns,
	float elapsedSeconds)
{
	constexpr float kMinimumSampleIntervalSeconds = 0.001f;
	constexpr float kMaximumSampleIntervalSeconds = 0.2f;
	constexpr float kOrientationPeriodTurns = 0.5f;
	constexpr float kHalfOrientationPeriodTurns = kOrientationPeriodTurns * 0.5f;
	if (!std::isfinite(elapsedSeconds) || elapsedSeconds < kMinimumSampleIntervalSeconds
		|| elapsedSeconds > kMaximumSampleIntervalSeconds)
		return {};

	std::array<float, 3> change{};
	for (size_t axis = 0; axis < change.size(); ++axis)
	{
		float deltaTurns = currentTurns[axis] - previousTurns[axis];
		if (deltaTurns > kHalfOrientationPeriodTurns)
			deltaTurns -= kOrientationPeriodTurns;
		else if (deltaTurns < -kHalfOrientationPeriodTurns)
			deltaTurns += kOrientationPeriodTurns;
		const float axisSign = axis == 0 ? -1.0f : 1.0f;
		change[axis] = deltaTurns / elapsedSeconds * axisSign;
	}
	return change;
}

// The monitor visualizes the raw screen-relative values received from Android,
// independently of the axis remapping used by the VPAD.
inline std::array<double, 3> ToMonitorRadians(const TwinpadInputState& input)
{
	constexpr double kInt16Turns = 32768.0;
	constexpr double kTau = 6.283185307179586;
	return {
		static_cast<double>(input.yaw) / kInt16Turns * kTau,
		static_cast<double>(input.pitch) / kInt16Turns * kTau,
		static_cast<double>(input.roll) / kInt16Turns * kTau,
	};
}

inline std::array<double, 2> ToMonitorEndPoint(const TwinpadInputState& input, size_t axis, double radius)
{
	const double radians = ToMonitorRadians(input)[axis];
	return {std::cos(radians) * radius * 0.86, -std::sin(radians) * radius * 0.86};
}

inline bool RollPointsRight(const TwinpadInputState& input)
{
	constexpr int kNearZeroTurns = 2048;
	if (std::abs(static_cast<int>(input.roll)) > kNearZeroTurns)
		return true;
	const auto endpoint = ToMonitorEndPoint(input, 2, 1.0);
	return endpoint[0] > 0.0 && std::abs(endpoint[0]) >= std::abs(endpoint[1]);
}
}

enum class TwinpadPixelFormat : uint8_t
{
	Rgb8,
	Rgb8Srgb,
	Rgba8,
	Rgba8Srgb,
};

class TwinpadClient
{
public:
	static TwinpadClient& Instance();
	TwinpadInputState GetInputState() const;
	TwinpadInputState GetGyroMonitorState() const;
	void SetGyroMonitorEnabled(bool enabled) { m_gyroMonitorEnabled.store(enabled); }
	bool IsGyroMonitorEnabled() const { return m_gyroMonitorEnabled.load(); }
	static bool GetCurrentQr(TwinpadQr& qr);
	bool Start(bool captureGamePad);
	void Stop();
	void SetTitleActive(bool active, bool connectAutomatically = false);
	void RequestConnection();
	void CancelConnectionRequest();
	bool IsConnected() const { return m_connected.load(); }
	bool IsConnectionRequested() const { return m_connectionRequested.load(); }
	bool IsCaptureEnabled() const { return m_captureGamePad.load(); }
	uint64_t GetLastFrameSentMs() const { return m_lastFrameSentMs.load(); }
	void SetCaptureGamePad(bool enabled) { m_captureGamePad.store(enabled); }
	void PublishGamePadFrame(
		std::vector<uint8_t>&& pixels,
		uint16_t width,
		uint16_t height,
		TwinpadPixelFormat format = TwinpadPixelFormat::Rgb8);
	void SetCaptureFps(uint8_t fps);
	void SetHardwareAccelerationEnabled(bool enabled) { m_hardwareAccelerationEnabled.store(enabled); }
	bool IsHardwareAccelerationEnabled() const { return m_hardwareAccelerationEnabled.load(); }
	bool IsHardwareEncoderActive();
	uint8_t GetCaptureFps() const { return m_captureFps.load(); }
	bool EncodeGamePadFrame(const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height, TwinpadEncodedFrame& output);
	void ResetVideoEncoder();
	void PublishAudioBlock(const int16_t* samples, size_t samplesPerChannel, uint8_t channels);
	void RecordVideoMetric(TwinpadVideoMetricStage stage, uint64_t elapsedNanoseconds);
	void RecordVideoEvent(TwinpadVideoMetricEvent event);
	void FlushVideoMetricsIfDue();

private:
	friend void TwinpadClient_base64url_test();
	friend void TwinpadClient_hardware_encoder_test();
	friend void TwinpadGyroMonitor_test();
	TwinpadClient() = default;
	~TwinpadClient();
	bool ConnectPrimary();
	bool ConnectAudio();
	void AudioLoop();
	void FrameLoop();
	void InputLoop();
	bool SendLine(std::intptr_t socket, const std::string& line);
	static std::string Base64Url(const uint8_t* data, size_t size);

	std::atomic_bool m_connected{false};
	std::atomic_bool m_captureGamePad{false};
	std::atomic_bool m_running{false};
	std::atomic_bool m_titleActive{false};
	std::atomic_bool m_connectionRequested{false};
	std::atomic_bool m_gyroMonitorEnabled{false};
	std::atomic<uint64_t> m_frameNumber{0};
	std::atomic<uint64_t> m_audioSequence{0};
	std::atomic<uint64_t> m_lastFrameSentMs{0};
	std::atomic<uint64_t> m_lastFrameQueuedUs{0};
	std::atomic<uint64_t> m_lastCaptureMs{0};
	std::atomic<uint8_t> m_captureFps{30};
	std::atomic_bool m_hardwareAccelerationEnabled{false};
	mutable std::mutex m_inputMutex;
	TwinpadInputState m_inputState{};
	TwinpadInputState m_gyroMonitorState{};
	bool m_hasGyroMonitorState{};
	bool m_hasInputSeq{};
	std::string m_receiveBuffer;
	std::mutex m_lifecycleMutex;
	std::mutex m_audioQueueMutex;
	std::condition_variable m_audioQueueChanged;
	std::deque<std::vector<int16_t>> m_audioQueue;
	std::mutex m_frameQueueMutex;
	std::condition_variable m_frameQueueChanged;
	std::vector<uint8_t> m_pendingFrame;
	std::vector<uint8_t> m_i420Buffer;
	uint16_t m_pendingFrameWidth{};
	uint16_t m_pendingFrameHeight{};
	TwinpadPixelFormat m_pendingFrameFormat{TwinpadPixelFormat::Rgb8};
	bool m_hasPendingFrame{};
	std::intptr_t m_primarySocket{-1};
	std::intptr_t m_audioSocket{-1};
	std::mutex m_primarySocketMutex;
	std::mutex m_audioSocketMutex;
	std::thread m_audioThread;
	std::thread m_frameThread;
	std::thread m_inputThread;
	std::mutex m_encoderMutex;
	void* m_videoEncoder{};
	void* m_hardwareVideoEncoder{};
	bool m_encoderHardwareActive{};
	bool m_encoderHardwarePreference{};
	uint16_t m_encoderWidth{};
	uint16_t m_encoderHeight{};
	uint8_t m_encoderFps{};
	uint64_t m_encoderFrameIndex{};
	bool m_forceNextIdr{true};
	uint64_t m_lastIdrTimestampUs{};
	uint64_t m_encoderEpochUs{};
	std::array<std::atomic<uint64_t>, static_cast<size_t>(TwinpadVideoMetricStage::COUNT)> m_videoMetricCounts{};
	std::array<std::atomic<uint64_t>, static_cast<size_t>(TwinpadVideoMetricStage::COUNT)> m_videoMetricNanoseconds{};
	std::array<std::atomic<uint64_t>, static_cast<size_t>(TwinpadVideoMetricStage::COUNT)> m_videoMetricMaxNanoseconds{};
	std::array<std::atomic<uint64_t>, static_cast<size_t>(TwinpadVideoMetricEvent::COUNT)> m_videoMetricEvents{};
	std::atomic<uint64_t> m_lastVideoMetricLogMs{};
};
