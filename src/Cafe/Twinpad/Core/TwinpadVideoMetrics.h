#pragma once

#include "Cafe/Twinpad/Core/TwinpadTypes.h"

#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>

// Metricas por etapa del pipeline de video del modulo Twinpad (CR-053):
// contadores atomicos sin bloqueo y una ventana de reporte compartida con
// el canal primario (la MISMA pasada de exchange alimenta el log y la
// linea `video_stage_stats`). Transplante del flush del cliente original.

namespace twinpad_video_metrics
{

constexpr uint64_t kReportIntervalMs = 5'000;
// CR-053: las rafagas de eventos no deben dominar la ventana; mas alla
// del tope el contador se recorta al reportarlo.
constexpr uint64_t kReportCountCap = 100000;

struct FlushResult
{
	std::string summary;      // "[TwinpadPerf] window_ms=..." (log Force)
	std::string stagesJson;
	std::string eventsJson;
};

class TwinpadVideoMetrics
{
public:
	void RecordStage(TwinpadVideoMetricStage stage, uint64_t elapsedNanoseconds);
	void RecordEvent(TwinpadVideoMetricEvent event);

	// Ventana de 5 s (CR-053): hace exchange(0) de TODOS los contadores
	// (una sola pasada) y devuelve el summary del log con las lineas
	// stagesJson/eventsJson separadas; el cliente decide si encola la
	// linea `video_stage_stats` (recordar: el canal media nunca la ve).
	// Devuelve falso si la ventana no vence todavia este tick.
	bool FlushIfDue(FlushResult& out);

private:
	std::array<std::atomic<uint64_t>, static_cast<size_t>(TwinpadVideoMetricStage::COUNT)> m_stageCounts{};
	std::array<std::atomic<uint64_t>, static_cast<size_t>(TwinpadVideoMetricStage::COUNT)> m_stageNanoseconds{};
	std::array<std::atomic<uint64_t>, static_cast<size_t>(TwinpadVideoMetricStage::COUNT)> m_stageMaxNanoseconds{};
	std::array<std::atomic<uint64_t>, static_cast<size_t>(TwinpadVideoMetricEvent::COUNT)> m_eventCounts{};
	std::atomic<uint64_t> m_lastLogMs{};
	std::mutex m_mutex;
};

} // namespace twinpad_video_metrics
