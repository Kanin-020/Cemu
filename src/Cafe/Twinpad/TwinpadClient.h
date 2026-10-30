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
	SCALE,
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

// CR-044: transición del teclado del juego pendiente de aplicar por el HLE
// `swkbd` en su hilo de render. `text` son unidades de código UTF-16 (el form
// del swkbd guarda una por carácter).
struct TwinpadKeyboardPending
{
	enum class Action : uint8_t
	{
		Update,
		Confirm,
		Cancel,
	};
	Action action{Action::Update};
	std::vector<uint16_t> text;
};

namespace TwinpadMotionMapper
{
// Android already sends AndroidDSU reference axes; Cemu only converts units.
inline std::array<float, 3> ToCemuGyroRadiansPerSecond(const TwinpadInputState& input)
{
	constexpr float kReferenceGyroSensitivity = 0.9f;
	return {input.gyro[0] * kReferenceGyroSensitivity,
		input.gyro[1] * kReferenceGyroSensitivity,
		input.gyro[2] * kReferenceGyroSensitivity};
}

inline std::array<float, 3> ToCemuAcceleration(const TwinpadInputState& input)
{
	constexpr float kReferenceAccelerationScale = 9.8066f / 100.0f;
	return {input.accel[0] * kReferenceAccelerationScale,
		input.accel[1] * kReferenceAccelerationScale,
		input.accel[2] * kReferenceAccelerationScale};
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
	// CR-036: el DRC de títulos como Wind Waker HD usa BGRA; sin estos
	// formatos el frame se rechazaba en silencio y la Segunda pantalla
	// quedaba sin video.
	Bgra8,
	Bgra8Srgb,
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
	void SetCaptureBitrate(uint32_t bitrateKbps);
	// CR-030: resolución de emisión del GamePad (DRC). Solo afecta al frame
	// publicado; la resolución de render del juego en TV no cambia.
	void SetCaptureResolution(uint32_t width, uint32_t height);
	uint32_t GetCaptureRequestWidth() const { return m_captureWidth.load(); }
	uint32_t GetCaptureRequestHeight() const { return m_captureHeight.load(); }
	void SetHardwareAccelerationEnabled(bool enabled) { m_hardwareAccelerationEnabled.store(enabled); }
	bool IsHardwareAccelerationEnabled() const { return m_hardwareAccelerationEnabled.load(); }
	bool IsHardwareEncoderActive();
	uint32_t GetCaptureBitrate() const { return m_captureBitrate.load(); }
	bool EncodeGamePadFrame(const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height, TwinpadEncodedFrame& output);
	void ResetVideoEncoder();
	static std::vector<uint8_t> ScaleRgb(
		const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height, uint16_t targetWidth, uint16_t targetHeight);
	void PublishAudioBlock(const int16_t* samples, size_t samplesPerChannel, uint8_t channels);
	// CR-033: patrón de VPADControlMotor hacia el teléfono; length 0 = parar.
	void PublishRumble(const uint8_t* pattern, uint8_t length);
	// CR-038: demanda vigente del video de la cámara del teléfono; el HLE
	// `camera` la emite en las transiciones de instancias abiertas.
	void PublishCameraRequest(bool on);
	static std::string CameraRequestLine(bool on);
	// CR-035: demanda vigente del micrófono del teléfono; el HLE `mic` la
	// emite al abrir/cerrar con MICOpen/MICClose. El teléfono solo graba
	// con demanda vigente (bajo demanda).
	void PublishMicRequest(bool on);
	static std::string MicRequestLine(bool on);
	// CR-035: consumo por el HLE `mic` (s16 mono host-order); false si no
	// hay datos frescos del teléfono (el HLE usa sus caminos existentes).
	bool ConsumeMicSamples(int16_t* dst, size_t count);
	// CR-044: demanda vigente del teclado del juego; el HLE `swkbd` la emite
	// al aparecer/desaparecer y en cada cambio del texto del form. `text` son
	// unidades de código UTF-16 (se codifican base64url UTF-8 en el wire).
	void PublishKeyboardRequest(bool on, const uint16_t* text, uint16_t count, uint16_t maxLength);
	static std::string KeyboardRequestLine(bool on, const std::string& textBase64Url, uint16_t maxLength);
	// CR-044: `keyboard_input` del teléfono (action/text base64url); falso si
	// la línea es incoherente (E4, se ignora). Encola para el hilo del swkbd.
	bool FeedKeyboardInput(std::string_view line);
	// CR-044: consumo por el HLE `swkbd` en su hilo de render; false sin pendientes.
	bool ConsumeKeyboardInput(TwinpadKeyboardPending& out);
	void RecordVideoMetric(TwinpadVideoMetricStage stage, uint64_t elapsedNanoseconds);
	void RecordVideoEvent(TwinpadVideoMetricEvent event);
	void FlushVideoMetricsIfDue();

private:
	friend void TwinpadClient_base64url_test();
	friend void TwinpadClient_camera_request_test();
	friend void TwinpadClient_mic_test();
	friend void TwinpadClient_keyboard_test();
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
	// CR-038: último estado de demanda de cámara; ConnectPrimary lo reenvía
	// tras game_announce si el IPC vivió una reconexión.
	std::atomic_bool m_cameraDemandOn{false};
	// CR-035: último estado de demanda de micrófono (idem cámara).
	std::atomic_bool m_micDemandOn{false};
	// CR-044: último estado de demanda de teclado vigente; ConnectPrimary lo
	// reenvía tras game_announce si el IPC vivió una reconexión.
	std::atomic_bool m_keyboardDemandOn{false};
	std::atomic_uint16_t m_keyboardDemandMaxLength{0};
	std::atomic<uint64_t> m_frameNumber{0};
	std::atomic<uint64_t> m_audioSequence{0};
	std::atomic<uint64_t> m_lastFrameSentMs{0};
	std::atomic<uint64_t> m_lastCaptureMs{0};
	std::atomic<uint32_t> m_captureBitrate{8000}; // kbps (CR-027: default 8000, clamped en SetCaptureBitrate)
	// CR-030: target de resolución del DRC (0/0 = nativo, sin escala).
	std::atomic<uint32_t> m_captureWidth{0};
	std::atomic<uint32_t> m_captureHeight{0};
	// CR-029: sin pacing ni tope de fps. La cadencia del encoder es la que
	// sostenga captura+encode (CR-030: el rate-control usa la cadencia REAL
	// medida; con un nominal fijo de 60, emitir a N fps reales acota el
	// stream a N/60 del bitrate objetivo).
	static constexpr uint8_t kEncoderNominalFps = 60; // estimación inicial
	// CR-036: si no llega un frame nuevo en este intervalo, se repite el último.
	static constexpr int kFrameRepeatIntervalMs = 100;
	std::vector<uint8_t> m_lastFrameRgb; // último frame RGB8 ya escalado (solo FrameLoop)
	uint16_t m_lastFrameWidth{};
	uint16_t m_lastFrameHeight{};
	static constexpr size_t kCadenceWindowFrames = 12;
	std::vector<uint64_t> m_encodeStampsUs; // ventana rodante detrás de m_encoderMutex
	uint16_t m_encoderNominalFps{}; // cadencia configurada en el encoder vigente
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
	// CR-035: cola s16 host-order del micrófono del teléfono, drop-old a
	// 1 s (32000 muestras @ 32 kHz). ConsumeMicSamples exige frescura.
	static constexpr size_t kMicQueueMaxSamples = 32000; // 1 s @ 32 kHz
	static constexpr uint64_t kMicFreshIntervalMs = 250;
	std::mutex m_micSamplesMutex;
	std::deque<int16_t> m_micSamples;
	std::atomic<uint64_t> m_lastMicChunkMs{0};
	bool FeedMicChunk(std::string_view line);
	// CR-044: cola (drop-old) de transiciones pendientes que aplica el HLE
	// `swkbd` en su hilo de render, y el texto base64url de la demanda vigente.
	static constexpr size_t kKeyboardPendingMax = 8;
	std::mutex m_keyboardMutex;
	std::deque<TwinpadKeyboardPending> m_keyboardPending;
	std::string m_keyboardDemandTextBase64Url;
	std::mutex m_lifecycleMutex;
	std::mutex m_audioQueueMutex;
	std::condition_variable m_audioQueueChanged;
	std::deque<std::vector<int16_t>> m_audioQueue;
	std::mutex m_frameQueueMutex;
	std::condition_variable m_frameQueueChanged;
	std::vector<uint8_t> m_pendingFrame;
	// CR-043: buffer de conversión YUV por frame (I420 para OpenH264, NV12
	// para los encoders hardware QSV/VAAPI/AMF/VideoToolbox). Tamaño
	// idéntico: width*height*3/2.
	std::vector<uint8_t> m_yuvBuffer;
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
	uint32_t m_encoderBitrateKbps{};
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
