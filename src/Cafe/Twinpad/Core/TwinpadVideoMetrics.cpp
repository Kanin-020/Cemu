#include "Cafe/Twinpad/Core/TwinpadVideoMetrics.h"

#include "Common/precompiled.h"

#include <chrono>
#include <string>

namespace twinpad_video_metrics
{

void TwinpadVideoMetrics::RecordStage(TwinpadVideoMetricStage stage, uint64_t elapsedNanoseconds)
{
	const size_t index = static_cast<size_t>(stage);
	m_stageCounts[index].fetch_add(1, std::memory_order_relaxed);
	m_stageNanoseconds[index].fetch_add(elapsedNanoseconds, std::memory_order_relaxed);
	uint64_t maximum = m_stageMaxNanoseconds[index].load(std::memory_order_relaxed);
	while (maximum < elapsedNanoseconds
		&& !m_stageMaxNanoseconds[index].compare_exchange_weak(maximum, elapsedNanoseconds, std::memory_order_relaxed))
	{
	}
}

void TwinpadVideoMetrics::RecordEvent(TwinpadVideoMetricEvent event)
{
	m_eventCounts[static_cast<size_t>(event)].fetch_add(1, std::memory_order_relaxed);
}

bool TwinpadVideoMetrics::FlushIfDue(FlushResult& out)
{
	const uint64_t nowMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	uint64_t previousMs = m_lastLogMs.load(std::memory_order_relaxed);
	if (nowMs - previousMs < kReportIntervalMs
		|| !m_lastLogMs.compare_exchange_strong(previousMs, nowMs, std::memory_order_relaxed))
		return false;

	static constexpr const char* stageNames[] = {
		"readback_copy", "readback_ready", "pixel_convert", "scale", "i420_convert",
		"hardware_encoder", "openh264", "encode_pipeline", "base64_json", "ipc_send",
	};
	static constexpr const char* eventNames[] = {
		"drc_callback", "capture_accepted", "readback_queued", "readback_completed",
		"readback_slot_busy", "frame_queued", "frame_encoded", "frame_sent",
	};
	std::string summary = "[TwinpadPerf] window_ms=" + std::to_string(kReportIntervalMs);
	// CR-053: la MISMA pasada de exchange(...,0) alimenta el log y la linea
	// `video_stage_stats` del HUD (una sola pasada de contadores; sin
	// carrera entre ventanas).
	std::string stagesJson;
	for (size_t index = 0; index < m_stageCounts.size(); ++index)
	{
		const uint64_t count = m_stageCounts[index].exchange(0, std::memory_order_relaxed);
		const uint64_t totalNs = m_stageNanoseconds[index].exchange(0, std::memory_order_relaxed);
		const uint64_t maxNs = m_stageMaxNanoseconds[index].exchange(0, std::memory_order_relaxed);
		if (count == 0)
			continue;
		const std::string avgUs = std::to_string(totalNs / count / 1000);
		const std::string maxUs = std::to_string(maxNs / 1000);
		summary += " " + std::string(stageNames[index]) + "_n=" + std::to_string(count)
			+ "_avg_us=" + avgUs
			+ "_max_us=" + maxUs;
		if (!stagesJson.empty())
			stagesJson += ',';
		stagesJson += "{\"stage\":\"";
		stagesJson += stageNames[index];
		stagesJson += "\",\"count\":" + std::to_string(count > kReportCountCap ? kReportCountCap : count);
		stagesJson += ",\"avg_us\":" + avgUs;
		stagesJson += ",\"max_us\":" + maxUs;
		stagesJson += '}';
	}
	std::string eventsJson;
	for (size_t index = 0; index < m_eventCounts.size(); ++index)
	{
		const uint64_t count = m_eventCounts[index].exchange(0, std::memory_order_relaxed);
		if (count == 0)
			continue;
		summary += " " + std::string(eventNames[index]) + "=" + std::to_string(count);
		if (!eventsJson.empty())
			eventsJson += ',';
		eventsJson += "{\"event\":\"";
		eventsJson += eventNames[index];
		eventsJson += "\",\"count\":" + std::to_string(count > kReportCountCap ? kReportCountCap : count);
		eventsJson += '}';
	}
	out.summary = std::move(summary);
	out.stagesJson = std::move(stagesJson);
	out.eventsJson = std::move(eventsJson);
	return true;
}

} // namespace twinpad_video_metrics
