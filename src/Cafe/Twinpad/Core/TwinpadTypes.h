#pragma once

#include <array>
#include <cstdint>
#include <cmath>
#include <string>
#include <vector>

// Tipos compartidos del módulo Twinpad: métricas, frame/encoder, QR, input,
// motion, teclado y el mapper de unidades. Los consumidores externos
// (gui, HLE, adaptadores de input/capture) solo necesitan este header más
// TwinpadClient.h; los detalles de transporte/encode son internos.

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

// CR-054: access unit ya codificada más los metadatos de envío. La línea
// NDJSON de `game_frame` (fallback del canal binario) se construye solo si
// el canal media no existe; en el camino normal el anexo viaja crudo.
struct TwinpadQueuedAu
{
	// CR-054: anexo crudo del AU. Vacío cuando el elemento es una línea
	// NDJSON pura (p. ej. `video_stage_stats` de CR-053, que solo viaja por
	// el canal primario y nunca por el binario).
	std::vector<uint8_t> annexB;
	std::string ndjsonLine;
	uint64_t gfn{};
	uint64_t timestampUs{};
	uint16_t width{};
	uint16_t height{};
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

// CR-045: muestra de motion recibida, pendiente de procesar por el VPAD en
// su hilo de input. Los ejes son los del marco AndroidDSU (CR-037); el
// mapeo a unidades Cemu se aplica en el drenaje, no en la encolada.
struct TwinpadMotionSample
{
	uint64_t timestampUs{};
	uint32_t seq{};
	std::array<float, 3> accel{};
	std::array<float, 3> gyro{};
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

inline const char* TwinpadFormatName(TwinpadPixelFormat format)
{
	switch (format)
	{
	case TwinpadPixelFormat::Rgb8: return "Rgb8";
	case TwinpadPixelFormat::Rgb8Srgb: return "Rgb8Srgb";
	case TwinpadPixelFormat::Rgba8: return "Rgba8";
	case TwinpadPixelFormat::Rgba8Srgb: return "Rgba8Srgb";
	case TwinpadPixelFormat::Bgra8: return "Bgra8";
	case TwinpadPixelFormat::Bgra8Srgb: return "Bgra8Srgb";
	default: return "?";
	}
}

// Android already sends AndroidDSU reference axes; Cemu only converts units.
namespace TwinpadMotionMapper
{
constexpr float kReferenceGyroSensitivity = 0.9f;
constexpr float kReferenceAccelerationScale = 9.8066f / 100.0f;

inline std::array<float, 3> ToCemuGyroRadiansPerSecond(const TwinpadInputState& input)
{
	return {input.gyro[0] * kReferenceGyroSensitivity,
		input.gyro[1] * kReferenceGyroSensitivity,
		input.gyro[2] * kReferenceGyroSensitivity};
}

inline float ToCemuGyroRadiansPerSecond(float value) { return value * kReferenceGyroSensitivity; }

inline std::array<float, 3> ToCemuAcceleration(const TwinpadInputState& input)
{
	return {input.accel[0] * kReferenceAccelerationScale,
		input.accel[1] * kReferenceAccelerationScale,
		input.accel[2] * kReferenceAccelerationScale};
}

inline float ToCemuAcceleration(float value) { return value * kReferenceAccelerationScale; }

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
