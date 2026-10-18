#pragma once

#include <atomic>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

struct TwinpadQr
{
	std::string uri;
	std::vector<std::string> modules;
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

class TwinpadClient
{
public:
	static TwinpadClient& Instance();
	TwinpadInputState GetInputState() const;
	static bool GetCurrentQr(TwinpadQr& qr);
	bool Start(bool captureGamePad);
	void Stop();
	void SetTitleActive(bool active, bool connectAutomatically = false);
	void RequestConnection();
	bool IsConnected() const { return m_connected.load(); }
	bool IsConnectionRequested() const { return m_connectionRequested.load(); }
	bool IsCaptureEnabled() const { return m_captureGamePad.load(); }
	uint64_t GetLastFrameSentMs() const { return m_lastFrameSentMs.load(); }
	void SetCaptureGamePad(bool enabled) { m_captureGamePad.store(enabled); }
	void PublishGamePadFrame(const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height);
	void PublishAudioBlock(const int16_t* samples, size_t samplesPerChannel, uint8_t channels);

private:
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
	std::atomic<uint64_t> m_frameNumber{0};
	std::atomic<uint64_t> m_audioSequence{0};
	std::atomic<uint64_t> m_lastFrameSentMs{0};
	std::atomic<uint64_t> m_lastFrameQueuedMs{0};
	std::atomic<uint64_t> m_lastCaptureMs{0};
	mutable std::mutex m_inputMutex;
	TwinpadInputState m_inputState{};
	bool m_hasInputSeq{};
	std::string m_receiveBuffer;
	std::mutex m_lifecycleMutex;
	std::mutex m_audioQueueMutex;
	std::condition_variable m_audioQueueChanged;
	std::deque<std::vector<int16_t>> m_audioQueue;
	std::mutex m_frameQueueMutex;
	std::condition_variable m_frameQueueChanged;
	std::vector<uint8_t> m_pendingFrame;
	uint16_t m_pendingFrameWidth{};
	uint16_t m_pendingFrameHeight{};
	bool m_hasPendingFrame{};
	std::intptr_t m_primarySocket{-1};
	std::intptr_t m_audioSocket{-1};
	std::mutex m_primarySocketMutex;
	std::mutex m_audioSocketMutex;
	std::thread m_audioThread;
	std::thread m_frameThread;
	std::thread m_inputThread;
};
