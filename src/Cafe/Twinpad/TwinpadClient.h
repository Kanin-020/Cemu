#pragma once

#include <array>
#include <atomic>
#include <condition_variable>
#include <chrono>
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
	bool motionValid{};
	uint64_t motionTimestampUs{};
	std::array<float, 3> accel{};
	std::array<float, 3> gyro{};
	float accelerometerMagnitude{};
	float accelerometerAcceleration{};
};

struct TwinpadFusedMotionState
{
	bool valid{};
	std::array<float, 3> accelerometer{};
	float accelerometerMagnitude{};
	float accelerometerAcceleration{};
	std::array<float, 3> gyroChange{};
	std::array<float, 3> gyroOrientation{};
	std::array<float, 9> attitude{};
};

namespace TwinpadMotionMapper
{
// Convert Android SI samples through the same axis/sign convention as AndroidDSU → Cemu DSU.
inline std::array<float, 3> ToCemuGyroRadiansPerSecond(const TwinpadInputState& input)
{
	constexpr float kReferenceGyroSensitivity = 0.9f;
	return {input.gyro[0] * kReferenceGyroSensitivity,
		-input.gyro[2] * kReferenceGyroSensitivity,
		input.gyro[1] * kReferenceGyroSensitivity};
}

inline std::array<float, 3> ToCemuAcceleration(const TwinpadInputState& input)
{
	constexpr float kReferenceAccelerationScale = 9.8066f / 100.0f;
	return {-input.accel[0] * kReferenceAccelerationScale,
		input.accel[2] * kReferenceAccelerationScale,
		-input.accel[1] * kReferenceAccelerationScale};
}

inline float ElapsedSeconds(uint64_t previousTimestampUs, uint64_t currentTimestampUs)
{
	if (currentTimestampUs <= previousTimestampUs)
		return 0.0f;
	return static_cast<float>(currentTimestampUs - previousTimestampUs) / 1'000'000.0f;
}

inline bool IsFinite(const TwinpadInputState& input)
{
	for (size_t axis = 0; axis < input.accel.size(); ++axis)
	{
		if (!std::isfinite(input.accel[axis]) || !std::isfinite(input.gyro[axis]))
			return false;
	}
	return true;
}

// The Cemu monitor visualizes the fused orientation reported by MotionSample in VPAD turns.
inline std::array<double, 2> ToMonitorEndPoint(float orientationTurns, double radius)
{
	constexpr double kTau = 6.283185307179586;
	const double radians = static_cast<double>(orientationTurns) * kTau;
	return {std::cos(radians) * radius * 0.86, -std::sin(radians) * radius * 0.86};
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
	bool HasFreshMotionInput() const;
	TwinpadFusedMotionState GetFusedMotionState() const;
	void SetFusedMotionState(const TwinpadFusedMotionState& state);
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
	TwinpadFusedMotionState m_fusedMotionState{};
	bool m_hasGyroMonitorState{};
	bool m_hasInputSeq{};
	std::chrono::steady_clock::time_point m_lastMotionReceived{};
	bool m_hasFreshMotionInput{};
	bool m_hasMotionTimestamp{};
	uint64_t m_lastMotionTimestampUs{};
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
