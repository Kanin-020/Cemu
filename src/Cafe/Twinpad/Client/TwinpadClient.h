#pragma once

#include "Cafe/Twinpad/Core/TwinpadTypes.h"
#include "Cafe/Twinpad/Channels/TwinpadMicChannel.h"
#include "Cafe/Twinpad/Channels/TwinpadKeyboardChannel.h"
#include "Cafe/Twinpad/Core/TwinpadVideoMetrics.h"

#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>


// Modular companions (implementation lives in the module TUs). The client
// only delegates to them from the wrappers below (SRP).
#include "Cafe/Twinpad/Video/TwinpadYuvConvert.h"
#include "Cafe/Twinpad/Core/TwinpadTransport.h"
#include "Cafe/Twinpad/Video/TwinpadH264.h"

class TwinpadCameraDecoder;
class TwinpadGyroMonitor;

//! Cemu face of Twinpad: the singleton that bridges gamepad input/output, audio,
//! and camera decode between this emulator process and the Twinpad host
//! (ports 8421 input / 8422 audio / 8423 QR / 8426 media binary / 8424 e2e).
class TwinpadClient
{
	friend void TwinpadGyroMonitor_test();
	friend void TwinpadMotionMapper_test();
	friend void TwinpadMotionQueue_test();
	friend void TwinpadClient_mic_test();
	friend void TwinpadClient_keyboard_test();
	friend void TwinpadClient_camera_request_test();
	friend void TwinpadClient_drc_scale_test();

  public:
	static constexpr uint32_t kPrimaryPort = 8421; // input/announce/rumble/game_frame/keyboard/camera etc. (NDJSON)
	static constexpr uint32_t kAudioPort = 8422;   // game_audio_chunk / game_announce-secondary (NDJSON)
	static constexpr uint32_t kCameraPort = 8423;  // camera stream (host side); this client only receives camera_frame elsewhere
	static constexpr uint32_t kMediaPort = 8426;   // binary media (CR-054): size-prefixed 420 NV12 -> AU
	static constexpr uint32_t kE2eCaptureIpcPort = 8424; // Test-058: e2e hook capture IPC probe

	// Contract of the binary media header (CR-054), 32 bytes, little endian.
	static constexpr uint32_t kMediaMagic = 0x5457;     // 'TW'
	static constexpr uint16_t kMediaVersion = 1;
	static constexpr uint16_t kMediaHeaderSize = 32;

	static constexpr size_t kMicQueueMaxSamples = 32000;   // ~1 s de PCM mono 32 kHz (saturar y dropear mas viejo)
	static constexpr size_t kMicChunkByteLimit = 64;       // base64url de <= 48 bytes PCM por linea NDJSON
	static constexpr size_t kMicBytesPerChunk = 48;        // PCM s16le mono: 24 muestras por chunk
	static constexpr int kMicChunkSendIntervalMs = 5;      // ~4.8 kHz de puertos minima para el demand pipeline
	static constexpr size_t kKeyboardPendingMax = 8;       // maximo de eventos pendientes en la cola del IME
	static constexpr size_t kMicRequestResendIntervalUs = 250000; // 250 ms entre reenvios de mic_request
	static constexpr int kVideoMetricLogIntervalMs = 5000; // ventana del summary de metricas de video
	static constexpr uint32_t kRetryIntervalMs = 2000;     // backoff de reconexion (input loop)
	static constexpr int kFrameRepeatIntervalMs = 100;     // ventana de repeticion del ultimo frame (CR-036)
	static constexpr uint32_t kEncoderNominalFps = 60;     // cadencia nominal del encode (CR-029)
	static constexpr uint32_t kIdrIntervalMs = 2000;       // periodo maximo entre IDR
	static constexpr size_t kMotionQueueMaxSamples = 32;   // cola de muestras motion del input loop (CR-045)
	static constexpr size_t kVideoSendQueueMaxLines = 4;   // drop-old del pipeline de video (CR-054)
	static constexpr uint32_t kGyroLogReportInterval = 10; // cada N seq de motion valido, linea de diagnostico

	using TwinpadInputStateFn = void (*)(const TwinpadInputState&);

	// CR-1a (desacoplamiento): el TU core enlaza el predicado a
	// CafeSystem::IsTitleRunning(); los tests pueden inyectar un stub.
	void SetTitleRunningPredicate(std::function<bool()> predicate) { m_titleRunningPredicate = std::move(predicate); }

	//! Thread-safe access to the singleton instance.
	static TwinpadClient& Instance();

	~TwinpadClient();

	//! Starts the four service loops (input, frame, video-send, audio). Idempotent.
	bool Start(bool captureGamePad);
	//! Joins every loop and closes every socket. Blocking; idempotent.
	void Stop();

	//! Called by the title menu when the host app connects / disconnects a
	//! specific game. If connectAutomatically, boots the client on demand.
	void SetTitleActive(bool active, bool connectAutomatically = false);

	//! Request a connection with the host once the game has been announced
	//! to the Twinpad host over the primary port 8421.
	void RequestConnection();
	void CancelConnectionRequest();

	// --- Video metrics ---------------------------------------------------------
	//! State of the hardware encoder (true while the HW path is in use).
	bool IsHardwareEncoderActive();
	//! Reset both encoders; next frame builds a new one from scratch.
	void ResetVideoEncoder();
	//! (Re)establish the target bitrate in kbps (CR-027) or run fallback.
	void SetCaptureBitrate(uint32_t bitrateKbps);
	//! (Re)instate the target resolution of the DRC stream (CR-030).
	void SetCaptureResolution(uint32_t width, uint32_t height);
	//! Force a new IDR on the next frame encoded (CR-051).
	void RequestKeyframe();

	// --- Camera request --------------------------------------------------------
	//! Canonical request line for the camera demand. Also used at reconnect.
	static std::string CameraRequestLine(bool on);
	//! Store + forward the current camera demand to the host; safe pre-connect.
	void PublishCameraRequest(bool on);

	// --- Mic request -----------------------------------------------------------
	//! Canonical request line of the mic demand (CR-035). Also used at reconnect.
	static std::string MicRequestLine(bool on);
	//! Store + forward the mic demand to the host; safe pre-connect.
	void PublishMicRequest(bool on);

	// --- Keyboard channel ------------------------------------------------------
	//! Canonical request line of the keyboard (CR-044). Also used at reconnect.
	static std::string KeyboardRequestLine(bool on, const std::string& textBase64Url, uint16_t maxLength);
	//! Store + forward the keyboard demand to the host; safe pre-connect.
	void PublishKeyboardRequest(bool on, const uint16_t* text, uint16_t count, uint16_t maxLength);
	//! Pops the oldest pending keyboard transition (strictly validated HLE IME).
	bool ConsumeKeyboardInput(TwinpadKeyboardPending& out);
	//! Validates one `mic_chunk` line and pushes s16 samples into the queue.
	bool FeedMicChunk(std::string_view line) { return m_micChannel.FeedMicChunk(line); }
	//! HLE `mic` consumer: fresh demand + fresh chunk window required (CR-035).
	bool ConsumeMicSamples(int16_t* dst, size_t count) { return m_micChannel.ConsumeMicSamples(dst, count); }
	//! Validates one `keyboard_input` line into the pending FIFO.
	bool FeedKeyboardInput(std::string_view line) { return m_keyboardChannel.FeedKeyboardInput(line); }
	// --- Rumble (NDJSON primary) ----------------------------------------------
	void PublishRumble(const uint8_t* pattern, uint8_t length);

	// --- Camera QR (port 8423) --------------------------------------------------
	//! Reads the camera QR from port 8423 (GET_QR handshake), safe pre-connect.
	static bool GetCurrentQr(TwinpadQr& qr);

	// --- Shared codec helpers (delegate to modules) ----------------------------
	//! base64url (no padding) shared by demands/frames (CR-035/CR-044/CR-054).
	static std::string Base64Url(const uint8_t* data, size_t size);
	//! CR-030 SRGB frame nearest-neighbor rescale; no upscale; empty on size error.
	static std::vector<uint8_t> ScaleRgb(const std::vector<uint8_t>& rgb, uint16_t width,
		uint16_t height, uint16_t targetWidth, uint16_t targetHeight);

	// --- Encoder entry (DRC pipeline) -------------------------------------------
	//! Encodes one RGB8 frame to a H.264 access unit (CR-020/CR-041 contract).
	bool EncodeGamePadFrame(const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height,
		TwinpadEncodedFrame& output);

	// --- Input consumers (VPAD HLE) -------------------------------------------
	//! Latest input state snapshot. Period 0.2 s stale window for motion.
	TwinpadInputState GetInputState() const;
	//! Gyro monitor's activity state from the same input (CR-045).
	bool HasFreshMotionInput() const;
	TwinpadFusedMotionState GetFusedMotionState() const;
	//! Motion fusion sink, driven by TwinpadGyroMonitor (consumes fresh input).
	void SetFusedMotionState(const TwinpadFusedMotionState& state);
	//! Drains this frame's pending motion samples (CR-045, one call per frame).
	std::vector<TwinpadMotionSample> DrainPendingMotionSamples();

	// --- Connection state (menu / HLE consumers) --------------------------------
	bool IsConnected() const { return m_connected.load(); }
	bool IsConnectionRequested() const { return m_connectionRequested.load(); }
	bool IsCaptureEnabled() const { return m_captureGamePad.load(); }
	uint64_t GetLastFrameSentMs() const { return m_lastFrameSentMs.load(); }
	void SetCaptureGamePad(bool enabled) { m_captureGamePad.store(enabled); }
	uint32_t GetCaptureBitrate() const { return m_captureBitrate.load(); }
	uint32_t GetCaptureRequestWidth() const { return m_captureWidth.load(); }
	uint32_t GetCaptureRequestHeight() const { return m_captureHeight.load(); }
	void SetHardwareAccelerationEnabled(bool enabled) { m_hardwareAccelerationEnabled.store(enabled); }
	bool IsHardwareAccelerationEnabled() const { return m_hardwareAccelerationEnabled.load(); }
	//! Motion samples captured by the input loop while the game polls motion.
	TwinpadInputState GetGyroMonitorState() const;
	//! TwinpadGyroMonitor activity state (CR-016/CR-017); opt-in.
	bool IsGyroMonitorEnabled() const { return m_gyroMonitorEnabled.load(); }
	//! Gyro monitor activity demand; enabled while the game polls motion.
	static void SetGyroMonitorEnabled(bool enabled)
	{
		Instance().m_gyroMonitorEnabled.store(enabled);
	}

	// --- Publishers (game data -> queues consumed by the loops) ---------------
	void PublishGamePadFrame(std::vector<uint8_t>&& pixels, uint16_t width, uint16_t height,
		TwinpadPixelFormat format = TwinpadPixelFormat::Rgb8);
	void PublishAudioBlock(const int16_t* samples, size_t samplesPerChannel, uint8_t channels);

	// --- Video metrics (CR-053; callers of the pipeline) -------------------------
	void RecordVideoMetric(TwinpadVideoMetricStage stage, uint64_t elapsedNanoseconds);
	void RecordVideoEvent(TwinpadVideoMetricEvent event);

	//! Camera QR decoder singleton, bound to port 8423 on the host side.

  private:
	TwinpadClient();

	// Thread loops --------------------------------------------------------------
	void InputLoop();     //!< Connects to 8421, parses input_report lines.
	void FrameLoop();     //!< RGB convert / scale / encode the DRC frame.
	void VideoSendLoop(); //!< Sends encoded AU to 8426 (or NDJSON fallback).
	void AudioLoop();     //!< Drains audio queue and sends to 8422.

	// Connection helpers -------------------------------------------------------
	bool ConnectPrimary(); //!< socket + game_announce + camera/mic/keyboard request resend
	bool ConnectAudio();   //!< Secondary socket on 8422.
	bool ConnectMedia();   //!< Binary 8426 (CR-054); optional (rich server only).
	void CloseMedia();     //!< Closes 8426 (idempotent).
	bool SendMediaFrame(const TwinpadQueuedAu& au); //!< One AU over the binary channel.
	bool SendLine(std::intptr_t socket, const std::string& line); //!< NDJSON single line + LF.
	bool SendBytes(std::intptr_t socket, const char* data, size_t size); //!< Raw bytes (media channel).

	// Encoder / pipeline helpers ------------------------------------------------
	void FlushVideoMetricsIfDue();
	//! Common motion state reset for disconnect/reconnect paths (CR-045).
	void ResetInputMotionState();

	// Members: ports / socket / queue state. All of them private.
	std::thread m_audioThread;
	std::thread m_frameThread;
	std::thread m_inputThread;
	std::thread m_videoSendThread;

	std::atomic<bool> m_running{false};
	std::mutex m_lifecycleMutex;

	// Verdad del titulo HLE activo; nulo = asumir activo (modo tests).
	bool TitleRunning() const { return !m_titleRunningPredicate || m_titleRunningPredicate(); }
	std::function<bool()> m_titleRunningPredicate;

	std::atomic<bool> m_titleActive{false};
	std::atomic<bool> m_connectionRequested{false};
	std::atomic<bool> m_captureGamePad{true};
	std::atomic<bool> m_connected{false};
	std::atomic<bool> m_mediaConnected{false};
	std::atomic<bool> m_gyroMonitorEnabled{false};
	std::atomic<bool> m_hardwareAccelerationEnabled{true};
	std::atomic<uint32_t> m_captureBitrate{4000};             // CR-027; server can override
	std::atomic<uint32_t> m_captureWidth{0};  // CR-030 native when 0
	std::atomic<uint32_t> m_captureHeight{0}; // CR-030 native when 0

	// Sockets (transport module); the transport owns socket semantics.
	std::intptr_t m_primarySocket = -1;
	std::intptr_t m_audioSocket = -1;
	std::intptr_t m_mediaSocket = -1;
	mutable std::mutex m_primarySocketMutex;
	mutable std::mutex m_audioSocketMutex;
	mutable std::mutex m_mediaSocketMutex;

	// Input state --------------------------------------------------------------
	// CR-045: la cola motion vive con el resto del estado de input.
	mutable std::mutex m_inputMutex;
	TwinpadInputState m_inputState;
	std::vector<TwinpadMotionSample> m_pendingMotionSamples;
	TwinpadInputState m_gyroMonitorState;
	TwinpadFusedMotionState m_fusedMotionState;
	std::chrono::steady_clock::time_point m_lastMotionReceived{};
	bool m_hasFreshMotionInput = false;
	bool m_hasMotionTimestamp = false;
	uint64_t m_lastMotionTimestampUs = 0;
	bool m_hasInputSeq = false;
	bool m_hasGyroMonitorState = false;

	// Camera / mic / keyboard demand state -------------------------------------
	std::atomic<bool> m_cameraDemandOn{false};
	std::atomic<bool> m_micDemandOn{false};
	std::atomic<bool> m_keyboardDemandOn{false};
	// CR-035: cuando el toggle cambia, el canal mic refresca su demand line
	// (see twinpad_mic_channel). It also bumps the sequence counter.
	std::atomic<uint64_t> m_micDemandToggleCount{0};

	// Queues -------------------------------------------------------------------
	std::mutex m_frameQueueMutex;
	std::condition_variable m_frameQueueChanged;
	std::vector<uint8_t> m_pendingFrame;
	uint16_t m_pendingFrameWidth = 0;
	uint16_t m_pendingFrameHeight = 0;
	TwinpadPixelFormat m_pendingFrameFormat = TwinpadPixelFormat::Rgb8;
	bool m_hasPendingFrame = false;
	std::vector<uint8_t> m_lastFrameRgb;
	uint16_t m_lastFrameWidth = 0;
	uint16_t m_lastFrameHeight = 0;

	std::mutex m_audioQueueMutex;
	std::condition_variable m_audioQueueChanged;
	std::deque<std::vector<int16_t>> m_audioQueue;

	std::mutex m_videoSendQueueMutex;
	std::condition_variable m_videoSendQueueChanged;
	std::deque<TwinpadQueuedAu> m_videoSendQueue;

	std::atomic<uint64_t> m_frameNumber{0};
	std::atomic<uint64_t> m_audioSequence{0};
	std::atomic<uint64_t> m_lastFrameSentMs{0};

	// Encoder -------------------------------------------------------------------
	std::mutex m_encoderMutex;
	// Legacy: software OpenH264 (void* ISVCEncoder) when HW is missing.
	void* m_videoEncoder = nullptr;
	// Hardware encoder backing structure (ffmpeg based, opaque here).
	std::unique_ptr<twinpad_h264::TwinpadHardwareEncoder> m_hardwareVideoEncoder;
	// Encoder configuration set by ResetVideoEncoder()/EncodeGamePadFrame().
	uint16_t m_encoderWidth = 0;
	uint16_t m_encoderHeight = 0;
	uint32_t m_encoderBitrateKbps = 0;
	uint32_t m_encoderNominalFps = 0;
	bool m_encoderHardwareActive = false;
	bool m_encoderHardwarePreference = true;
	uint64_t m_encoderFrameIndex = 0;
	std::atomic<bool> m_forceNextIdr{true};
	uint64_t m_lastIdrTimestampUs = 0;
	uint64_t m_encoderEpochUs = 0;

	// Receive state ---------------------------------------------------------------
	std::string m_receiveBuffer;

	// Encoder scratch --------------------------------------------------------------
	std::vector<uint8_t> m_yuvBuffer;         // CR-043: I420/NV12 staging buffer
	std::deque<uint64_t> m_encodeStampsUs;    // CR-030: cadencia real de emision
	static constexpr size_t kCadenceWindowFrames = 12;

	// Forwarded modules -----------------------------------------------------------
	twinpad_mic_channel::TwinpadMicChannel m_micChannel;
	twinpad_keyboard_channel::TwinpadKeyboardChannel m_keyboardChannel;
	twinpad_video_metrics::TwinpadVideoMetrics m_videoMetrics;

};

// Free-function tests (white-box friends). They live outside the class so
// they can reach both the client and its module members.
void TwinpadClient_base64url_test();
void TwinpadClient_h264_encoder_test();
void TwinpadClient_hardware_encoder_test();
void TwinpadGyroMonitor_test();
void TwinpadMotionMapper_test();
void TwinpadMotionQueue_test();
void TwinpadClient_mic_test();
void TwinpadClient_keyboard_test();
void TwinpadClient_camera_request_test();
void TwinpadClient_drc_scale_test();
