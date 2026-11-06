#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <functional>
#include <string>
#include <string_view>

// Canal de microfono del modulo Twinpad (CR-035): cola s16 host-order con
// drop-old de 1 s, validacion estricta de mic_chunk y telemetria de rechazos. Transplante del Feed/Consume del cliente original; el cliente
// solo inyecta el predicado de demanda vigente (m_micDemandOn es suyo) y el
// contador de toggles para el resumen de telemetria.

namespace twinpad_mic_channel
{

// CR-035: techo de 1 s de audio a 32 kHz; chunks frescos dentro de 250 ms.
constexpr size_t kMicQueueMaxSamples = 32000;
constexpr uint64_t kMicFreshIntervalMs = 250;

// CR-036: colchon de jitter. El telefono entrega bloques de 480 muestras cada
// ~15 ms mientras el AX pide 96 muestras cada 3 ms; sin colchon la cola se
// vacia en cada frontera de bloque y el frame que no encontraba muestras
// devolvia el micro al dispositivo del PC (silencios de 3 ms = cortes). Se
// arranca con 30 ms de colchon y se recorta a ese nivel por encima de 60 ms
// para acotar la latencia y la deriva de reloj.
constexpr size_t kMicJitterTargetSamples = 960;
constexpr size_t kMicJitterMaxSamples = 1920;

class TwinpadMicChannel
{
public:
	// Demanda vigente inyectada (el cliente la mantiene en m_micDemandOn);
	// ConsumeMicSamples exige demanda activa (CR-035: E1).
	void SetDemandPredicate(std::function<bool()> predicate);
	// Contador de toggles del publisher (el cliente lo incrementa y aqui
	// solo se intercambia en la ventana de telemetria).
	void SetToggleCounter(std::atomic<uint64_t>* toggles);

	// CR-035: mic_chunk estricto (samples 32..32000, bytes==samples*2 s16le,
	// base64url sin relleno); lo incoherente se ignora (E4) con log de
	// diagnostico <= 1 Hz por motivo.
	bool FeedMicChunk(std::string_view line);

	// CR-035: consumo por AX (~336 llamadas/s); exige demanda vigente y
	// chunks frescos (<= 250 ms). CR-036: mantiene un colchon de 30 ms y
	// rellena con silencio el tramo que falte, de forma que una cola corta no
	// devuelve el micro a la fuente del PC a mitad de frase.
	bool ConsumeMicSamples(int16_t* dst, size_t count);

	// Telemetria CR-035 sin throttle: tasas crudas por ventana (la usa el
	// flush de metricas del cliente al final de su ventana de 5 s).
	std::string BuildPerfSummary(uint64_t windowMs);

private:
	bool MicLogThrottled();
	void MicLogReject(const char* reason);
	void RecordMicQueueSample(size_t size);

	std::function<bool()> m_demandPredicate;
	std::atomic<uint64_t>* m_toggles{};

	std::deque<int16_t> m_samples;
	std::mutex m_samplesMutex;
	std::atomic<uint64_t> m_lastLogMs{};
	std::atomic<uint64_t> m_lastChunkMs{};
	std::atomic<uint64_t> m_rejectCount{};
	std::atomic<uint64_t> m_consumeFailCount{};
	std::atomic<uint64_t> m_padSamples{};
	std::atomic<bool> m_primed{};
	std::atomic<uint64_t> m_dropOldCount{};
	std::atomic<uint64_t> m_chunksValid{};
	std::atomic<uint64_t> m_consumes{};
	std::atomic<uint64_t> m_queueSizeLast{};
	std::atomic<uint64_t> m_queueSizeMax{};
};

} // namespace twinpad_mic_channel
