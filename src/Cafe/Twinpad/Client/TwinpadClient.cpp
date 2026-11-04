#include "Cafe/Twinpad/Client/TwinpadClient.h"
#include "Cafe/CafeSystem.h"

// Modulos internos que el TU nucleo instancia e invoca (el header del
// facade solo los declara por adelantado).
#include "Cafe/Twinpad/Channels/TwinpadMicChannel.h"
#include "Cafe/Twinpad/Channels/TwinpadKeyboardChannel.h"
#include "Cafe/Twinpad/Core/TwinpadVideoMetrics.h"
#include "Cafe/Twinpad/Video/TwinpadH264.h"
#include "Cafe/Twinpad/Core/TwinpadTransport.h"
#include "Cafe/Twinpad/Video/TwinpadYuvConvert.h"

#include "Common/precompiled.h"

#include <algorithm>

// ---------------------------------------------------------------------------
// TU nucleo del cliente Twinpad: singleton, estado de demanda (camera / mic /
// keyboard), wrappers sobre los modulos y getters de input. Los demas TUs
// (Connect/InputLoop/Encoder/Pipeline) cubren el resto de la clase; la API
// publica es la misma del monolito original.
// ---------------------------------------------------------------------------

TwinpadClient& TwinpadClient::Instance()
{
	static TwinpadClient client;
	return client;
}

TwinpadClient::TwinpadClient()
{
	// Modulos internos portados al facade (SRP): se instancian aqui para
	// que el header publique solo tipos, no detalles de canal.
	m_micChannel = std::make_unique<twinpad_mic_channel::TwinpadMicChannel>();
	m_keyboardChannel = std::make_unique<twinpad_keyboard_channel::TwinpadKeyboardChannel>();
	m_videoMetrics = std::make_unique<twinpad_video_metrics::TwinpadVideoMetrics>();
	// CR-035: la demanda del mic vive en el cliente (m_micDemandOn); el canal
	// consulta el predicado en cada consumo y comparte el contador de toggles
	// para su resumen de telemetria por ventana.
	m_micChannel->SetDemandPredicate([this] { return m_micDemandOn.load(std::memory_order_relaxed); });
	m_micChannel->SetToggleCounter(&m_micDemandToggleCount);
	// CR-1a: unico punto de contacto con CafeSystem del modulo; inyectado.
	SetTitleRunningPredicate([] { return CafeSystem::IsTitleRunning(); });
}

TwinpadClient::~TwinpadClient()
{
	Stop();
	ResetVideoEncoder();
}

// --- Helpers compartidos (delegan en modulos) -------------------------------

std::string TwinpadClient::Base64Url(const uint8_t* data, size_t size)
{
	return twinpad_transport::Base64Url(data, size);
}

std::vector<uint8_t> TwinpadClient::ScaleRgb(
	const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height, uint16_t targetWidth, uint16_t targetHeight) // CR-030
{
	return twinpad_yuv::ScaleRgb(rgb, width, height, targetWidth, targetHeight);
}

// --- Camera demand (CR-038) -------------------------------------------------

std::string TwinpadClient::CameraRequestLine(bool on)
{
	// CR-038: única forma canónica de la línea, compartida por la demanda
	// en vivo y el reenvío tras game_announce.
	return on ? "{\"type\":\"camera_request\",\"on\":true}" : "{\"type\":\"camera_request\",\"on\":false}";
}

void TwinpadClient::PublishCameraRequest(bool on)
{
	// CR-038: el último estado se recuerda aunque el IPC no esté vivo, para
	// que ConnectPrimary lo reenvíe al reconectar (el servidor lo entrega a
	// la sesión que seleccione el Plugin).
	m_cameraDemandOn.store(on);
	std::scoped_lock lock(m_primarySocketMutex);
	if (!m_connected.load() || m_primarySocket < 0)
		return;
	if (!SendLine(m_primarySocket, CameraRequestLine(on)))
		m_connected.store(false);
}

// --- Mic demand (CR-035) ----------------------------------------------------

std::string TwinpadClient::MicRequestLine(bool on)
{
	// CR-035: única forma canónica de la línea, compartida por la demanda
	// en vivo y el reenvío tras game_announce.
	return on ? "{\"type\":\"mic_request\",\"on\":true}" : "{\"type\":\"mic_request\",\"on\":false}";
}

void TwinpadClient::PublishMicRequest(bool on)
{
	// CR-035: bajo demanda — el teléfono no graba sin este estado vigente;
	// el HLE `mic` lo emite en las transiciones MICOpen/MICClose del juego.
	// La cola/validación/telemetría vive en twinpad_mic_channel; aquí solo
	// se publica la demanda y (si hay socket) se envía la línea canónica.
	if (m_micDemandOn.exchange(on) != on)
		m_micDemandToggleCount.fetch_add(1, std::memory_order_relaxed);
	m_micDemandOn.store(on);
	std::scoped_lock lock(m_primarySocketMutex);
	if (!m_connected.load() || m_primarySocket < 0)
		return;
	if (!SendLine(m_primarySocket, MicRequestLine(on)))
		m_connected.store(false);
}

// --- Keyboard demand (CR-044) -----------------------------------------------

std::string TwinpadClient::KeyboardRequestLine(bool on, const std::string& textBase64Url, uint16_t maxLength)
{
	// CR-044: única forma canónica de la línea, compartida por la demanda en
	// vivo y el reenvío tras game_announce; `off` no lleva texto ni techo.
	if (!on)
		return "{\"type\":\"keyboard_request\",\"on\":false}";
	return "{\"type\":\"keyboard_request\",\"on\":true,\"text\":\"" + textBase64Url
		+ "\",\"max_length\":" + std::to_string(maxLength) + "}";
}

void TwinpadClient::PublishKeyboardRequest(bool on, const uint16_t* text, uint16_t count, uint16_t maxLength)
{
	// CR-044: el último estado se recuerda en el canal (texto base64url y
	// techo vigentes) para que ConnectPrimary lo reenvíe al reconectar, como
	// cámara/micrófono.
	m_keyboardChannel->SetDemand(on, text, count, maxLength);
	std::scoped_lock lock(m_primarySocketMutex);
	if (!m_connected.load() || m_primarySocket < 0)
		return;
	const std::string textBase64Url
		= on ? m_keyboardChannel->GetKeyboardDemandTextBase64Url() : std::string();
	if (!SendLine(m_primarySocket,
			KeyboardRequestLine(on, textBase64Url, m_keyboardChannel->GetKeyboardDemandMaxLength())))
		m_connected.store(false);
}

bool TwinpadClient::ConsumeKeyboardInput(TwinpadKeyboardPending& out)
{
	// CR-044: transiciones FIFO del swkbd; el cap de pendientes viejas vive
	// en el canal (drop-old al encolar).
	return m_keyboardChannel->ConsumeKeyboardInput(out);
}

bool TwinpadClient::FeedKeyboardInput(std::string_view line)
{
	return m_keyboardChannel->FeedKeyboardInput(line);
}

// --- Mic (CR-035) ------------------------------------------------------------

bool TwinpadClient::FeedMicChunk(std::string_view line)
{
	return m_micChannel->FeedMicChunk(line);
}

bool TwinpadClient::ConsumeMicSamples(int16_t* dst, size_t count)
{
	return m_micChannel->ConsumeMicSamples(dst, count);
}

// --- Rumble (NDJSON primario) -----------------------------------------------

void TwinpadClient::PublishRumble(const uint8_t* pattern, uint8_t length)
{
	if (!pattern || length == 0)
		return;
	if (!m_connected.load())
		return;
	const std::string encoded = Base64Url(pattern, length);
	const std::string line = "{\"type\":\"rumble\",\"length\":" + std::to_string(length) + ",\"data\":\"" + encoded + "\"}";
	std::scoped_lock lock(m_primarySocketMutex);
	if (!m_connected.load() || m_primarySocket < 0)
		return;
	if (!SendLine(m_primarySocket, line))
		m_connected.store(false);
}

// --- Metricas de video (CR-053) ----------------------------------------------

void TwinpadClient::RecordVideoMetric(TwinpadVideoMetricStage stage, uint64_t elapsedNanoseconds)
{
	m_videoMetrics->RecordStage(stage, elapsedNanoseconds);
}

void TwinpadClient::RecordVideoEvent(TwinpadVideoMetricEvent event)
{
	m_videoMetrics->RecordEvent(event);
}

void TwinpadClient::FlushVideoMetricsIfDue()
{
	// Ventana de 5 s: la MISMA pasada de exchange alimenta el log de Cemu y
	// la linea `video_stage_stats` del HUD (modulo TwinpadVideoMetrics).
	twinpad_video_metrics::FlushResult result;
	if (!m_videoMetrics->FlushIfDue(result))
		return;
	cemuLog_log(LogType::Force, "{}", result.summary);

	// CR-053: la ventana cae también al HUD del teléfono vía
	// `video_stage_stats` por el canal primario; el server solo la reenvía
	// a la sesión con este juego seleccionado. Telemetría: si la cola
	// está al tope se descarta la línea MÁS VIEJA (la ventana de 5 s
	// siguiente repone los datos; nunca bloquea el encode).
	if (m_connected.load(std::memory_order_relaxed) && m_primarySocket >= 0)
	{
		// CR-075: diagnóstico del HUD — backend del encoder activo y resoluciones
		// de entrada (fuente del juego, pre-escala) y de salida (post-escala,
		// lo que viaja por el stream). 0 = sin fuente de video en la ventana.
		const bool hardwareActive = IsHardwareEncoderActive();
		std::string statsLine = "{\"type\":\"video_stage_stats\",";
		statsLine += "\"window_ms\":" + std::to_string(twinpad_video_metrics::kReportIntervalMs);
		statsLine += ",\"hw\":" + std::string(hardwareActive ? "true" : "false");
		statsLine += ",\"in_w\":" + std::to_string(m_lastSourceWidth);
		statsLine += ",\"in_h\":" + std::to_string(m_lastSourceHeight);
		statsLine += ",\"out_w\":" + std::to_string(m_lastFrameWidth);
		statsLine += ",\"out_h\":" + std::to_string(m_lastFrameHeight);
		statsLine += ",\"stages\":[]";
		if (!result.stagesJson.empty())
			statsLine.insert(statsLine.size() - 1, result.stagesJson);
		statsLine += ",\"events\":[]";
		if (!result.eventsJson.empty())
			statsLine.insert(statsLine.size() - 1, result.eventsJson);
		statsLine += '}';
		{
			std::scoped_lock lock(m_videoSendQueueMutex);
			if (m_videoSendQueue.size() == kVideoSendQueueMaxLines)
				m_videoSendQueue.pop_front();
			TwinpadQueuedAu stats;
			stats.ndjsonLine = std::move(statsLine);
			m_videoSendQueue.push_back(std::move(stats));
		}
		m_videoSendQueueChanged.notify_one();
	}

	// Telemetría CR-035 sin throttle: tasas crudas por ventana que reporta
	// el canal mic (accepted/consume/reject/drop/toggles/cola). Abrir la
	// brecha accepted vs consume delata feed corto o drain doble.
	const std::string micSummary
		= m_micChannel->BuildPerfSummary(twinpad_video_metrics::kReportIntervalMs);
	if (!micSummary.empty())
		cemuLog_log(LogType::Force, "{}", micSummary);
}

// --- Instrumentacion del video (CR-051/CR-027/CR-030) ------------------------

void TwinpadClient::RequestKeyframe() // CR-051: IDR inmediata para el nuevo suscriptor media
{
	m_forceNextIdr.store(true, std::memory_order_release);
	// TEMPORAL CR-065-diag: rastro de la solicitud del server.
	cemuLog_log(LogType::Force, "[TwinpadVideo] idr_request video_force_idr recibido");
}

void TwinpadClient::SetCaptureBitrate(uint32_t bitrateKbps) // CR-027: bitrate en caliente
{
	bitrateKbps = std::clamp(bitrateKbps, 1000u, 30000u);
	if (m_captureBitrate.exchange(bitrateKbps) == bitrateKbps)
		return;
	ResetVideoEncoder();
}

void TwinpadClient::SetCaptureResolution(uint32_t width, uint32_t height) // CR-030: resolución GamePad en caliente
{
	// CR-030: el encoding I420 exige dimensiones pares; el target impar
	// se reduce al par inferior. 0/0 desactiva la escala (nativo).
	width = (width >> 1) << 1;
	height = (height >> 1) << 1;
	if (width == 0 || height == 0)
	{
		m_captureWidth.store(0);
		m_captureHeight.store(0);
		return;
	}
	width = std::clamp(width, 16u, 3840u);
	height = std::clamp(height, 16u, 2160u);
	if (m_captureWidth.exchange(width) == width && m_captureHeight.exchange(height) == height)
		return;
	// La siguiente AU con dimensión efectiva distinta reinicia el encoder
	// (encoderConfigChanged); no hace falta rearmarlo aquí.
}

// --- Estado de input (VPAD/gyro monitor) ------------------------------------

TwinpadInputState TwinpadClient::GetInputState() const
{
	std::scoped_lock lock(m_inputMutex);
	return m_inputState;
}

TwinpadInputState TwinpadClient::GetGyroMonitorState() const
{
	std::scoped_lock lock(m_inputMutex);
	return m_hasGyroMonitorState ? m_gyroMonitorState : TwinpadInputState{};
}

bool TwinpadClient::HasFreshMotionInput() const
{
	std::scoped_lock lock(m_inputMutex);
	return m_hasFreshMotionInput
		&& std::chrono::steady_clock::now() - m_lastMotionReceived <= std::chrono::milliseconds(200);
}

TwinpadFusedMotionState TwinpadClient::GetFusedMotionState() const
{
	std::scoped_lock lock(m_inputMutex);
	return m_fusedMotionState;
}

void TwinpadClient::SetFusedMotionState(const TwinpadFusedMotionState& state)
{
	std::scoped_lock lock(m_inputMutex);
	m_fusedMotionState = state;
}

// CR-045: drenaje una vez por frame desde el hilo de input del VPAD; la cola
// vive detrás de m_inputMutex junto al resto del estado de motion.
std::vector<TwinpadMotionSample> TwinpadClient::DrainPendingMotionSamples()
{
	std::vector<TwinpadMotionSample> pending;
	{
		std::scoped_lock lock(m_inputMutex);
		pending.swap(m_pendingMotionSamples);
	}
	return pending;
}

// CR-045: reset común del estado de motion para los tres caminos (desconexión
// del input loop, reconexión y CancelConnectionRequest); sustituye el bloque
// repetido tres veces del monolito original.
void TwinpadClient::ResetInputMotionState()
{
	std::scoped_lock lock(m_inputMutex);
	m_inputState = TwinpadInputState{};
	m_pendingMotionSamples.clear();
	m_gyroMonitorState = TwinpadInputState{};
	m_fusedMotionState = TwinpadFusedMotionState{};
	m_lastMotionReceived = {};
	m_hasFreshMotionInput = false;
	m_hasMotionTimestamp = false;
	m_lastMotionTimestampUs = 0;
	m_hasInputSeq = false;
	m_hasGyroMonitorState = false;
}
