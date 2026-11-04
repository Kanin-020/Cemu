#include "Cafe/Twinpad/Channels/TwinpadMicChannel.h"

#include "Cafe/Twinpad/Core/TwinpadTransport.h"
#include "Cafe/Twinpad/Video/TwinpadYuvConvert.h"

#include "Common/precompiled.h"

#include <rapidjson/document.h>

#include <algorithm>
#include <chrono>
#include <string>

namespace twinpad_mic_channel
{	void TwinpadMicChannel::SetDemandPredicate(std::function<bool()> predicate)
	{
		m_demandPredicate = std::move(predicate);
	}


void TwinpadMicChannel::SetToggleCounter(std::atomic<uint64_t>* toggles)
{
	m_toggles = toggles;
}

bool TwinpadMicChannel::MicLogThrottled()
{
	const auto nowMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	uint64_t previousMs = m_lastLogMs.load();
	if (nowMs - previousMs < 1000)
		return false;
	return m_lastLogMs.compare_exchange_weak(previousMs, nowMs);
}

void TwinpadMicChannel::MicLogReject(const char* reason)
{
	m_rejectCount.fetch_add(1, std::memory_order_relaxed);
	if (MicLogThrottled())
		cemuLog_log(LogType::Force, "[TwinpadMic] linea descartada en Cemu (motivo: {})", reason);
}

void TwinpadMicChannel::RecordMicQueueSample(size_t size)
{
	m_queueSizeLast.store(size, std::memory_order_relaxed);
	uint64_t previousMax = m_queueSizeMax.load(std::memory_order_relaxed);
	while (static_cast<uint64_t>(size) > previousMax
		&& !m_queueSizeMax.compare_exchange_weak(previousMax, static_cast<uint64_t>(size), std::memory_order_relaxed))
	{
	}
}

bool TwinpadMicChannel::FeedMicChunk(std::string_view line)
{
	// CR-035: `mic_chunk` estricto — samples 32..32000 (1 ms..1 s @ 32 kHz)
	// y 2 bytes por muestra s16le; lo incoherente se ignora (E4).
	// Diagnostico: cada motivo de descarte se audita con log <= 1 Hz,
	// porque un rechazo en silencio es indistinguible desde el socket.
	rapidjson::Document document;
	document.Parse(line.data(), line.size());
	if (document.HasParseError() || !document.IsObject())
	{
		MicLogReject("json_invalido");
		return false;
	}
	if (!document.HasMember("type") || !document["type"].IsString()
		|| std::string_view(document["type"].GetString()) != "mic_chunk")
	{
		MicLogReject("tipo");
		return false;
	}
	if (!document.HasMember("samples") || !document["samples"].IsUint())
	{
		MicLogReject("sin_samples");
		return false;
	}
	const uint32_t samples = document["samples"].GetUint();
	if (samples < 32 || samples > 32000)
	{
		MicLogReject("samples_fuera_de_rango");
		return false;
	}
	if (!document.HasMember("data") || !document["data"].IsString())
	{
		MicLogReject("sin_data");
		return false;
	}
	const char* base64Data = document["data"].GetString();
	std::vector<uint8_t> decoded;
	if (!twinpad_transport::DecodeBase64Url(base64Data, document["data"].GetStringLength(), decoded))
	{
		MicLogReject("base64url_invalido");
		return false;
	}
	if (decoded.size() != static_cast<size_t>(samples) * 2)
	{
		MicLogReject("bytes_vs_samples_incoherente");
		return false;
	}
	std::deque<int16_t> chunk;
	for (size_t i = 0; i < decoded.size(); i += 2)
		chunk.push_back(static_cast<int16_t>(static_cast<uint16_t>(decoded[i]) | (static_cast<uint16_t>(decoded[i + 1]) << 8)));
	{
		// Drop-old: si la cola superaria 1 s de audio, se descarta lo mas antiguo.
		std::scoped_lock lock(m_samplesMutex);
		// CR-035: drop-old solo cuando la cola EXCEDERIA 1 s de audio. El
		// calculo anterior restaba en unsigned sin preguntar el total primero:
		// con la cola bajo el techo, size+chunk-32000 daba la vuelta a un
		// "overflow" enorme que vaciaba la cola a cada chunk (telemetria
		// TwinpadMicPerf: drop_old_n == accepted_n, cola_max = 1 chunk).
		const size_t totalSamples = m_samples.size() + chunk.size();
		if (totalSamples > kMicQueueMaxSamples)
		{
			const size_t overflow = totalSamples - kMicQueueMaxSamples;
			m_dropOldCount.fetch_add(1, std::memory_order_relaxed);
			m_samples.erase(m_samples.begin(), m_samples.begin() + static_cast<std::ptrdiff_t>(std::min(overflow, m_samples.size())));
		}
		m_samples.insert(m_samples.end(), chunk.begin(), chunk.end());
		RecordMicQueueSample(m_samples.size());
	}
	m_lastChunkMs.store(static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count()));
	// Diagnostico del ritmo de llegada: ~64 bloques de 480 muestras = 1 s.
	if (m_chunksValid.fetch_add(1) % 64 == 0 && MicLogThrottled())
		cemuLog_log(LogType::Force, "[TwinpadMic] mic_chunk aceptado (total={})", m_chunksValid.load());
	return true;
}

bool TwinpadMicChannel::ConsumeMicSamples(int16_t* dst, size_t count)
{
	if (!dst || count == 0 || (m_demandPredicate && !m_demandPredicate()))
	{
		MicLogReject("demanda_off");
		return false;
	}
	const auto nowMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	// CR-035: sin chunks frescos el HLE vuelve a sus caminos existentes
	// (dispositivo de entrada del PC / blow mic / silencio).
	const uint64_t last = m_lastChunkMs.load();
	if (nowMs > last + kMicFreshIntervalMs)
	{
		if (MicLogThrottled())
			cemuLog_log(LogType::Force, "[TwinpadMic] sin chunks frescos (edad_ms={})", nowMs - last);
		return false;
	}
	std::scoped_lock lock(m_samplesMutex);
	if (m_samples.size() < count)
	{
		m_consumeFailCount.fetch_add(1, std::memory_order_relaxed);
		RecordMicQueueSample(m_samples.size());
		if (MicLogThrottled())
			cemuLog_log(LogType::Force, "[TwinpadMic] cola insuficiente ({} de {} muestras)", m_samples.size(), count);
		return false;
	}
	for (size_t i = 0; i < count; ++i)
	{
		dst[i] = m_samples.front();
		m_samples.pop_front();
	}
	RecordMicQueueSample(m_samples.size());
	// Consumo por AX: ~336 AX frames por segundo alimentan las colas del juego.
	if (m_consumes.fetch_add(1) % 336 == 0 && MicLogThrottled())
		cemuLog_log(LogType::Force, "[TwinpadMic] consumo por AX activo (cola={})", m_samples.size());
	return true;
}

std::string TwinpadMicChannel::BuildPerfSummary(uint64_t windowMs)
{
	struct MicPerfSnapshot
	{
		uint64_t accepted = 0;
		uint64_t consumed = 0;
	};
	static MicPerfSnapshot micPrev{};
	const uint64_t acceptedNow = m_chunksValid.load(std::memory_order_relaxed);
	const uint64_t consumedNow = m_consumes.load(std::memory_order_relaxed);
	std::string summary = "[TwinpadMicPerf] window_ms=" + std::to_string(windowMs);
	summary += " accepted_n=" + std::to_string(acceptedNow - micPrev.accepted);
	summary += " consume_ok_n=" + std::to_string(consumedNow - micPrev.consumed);
	summary += " reject_n=" + std::to_string(m_rejectCount.exchange(0, std::memory_order_relaxed));
	summary += " consume_fail_n=" + std::to_string(m_consumeFailCount.exchange(0, std::memory_order_relaxed));
	summary += " drop_old_n=" + std::to_string(m_dropOldCount.exchange(0, std::memory_order_relaxed));
	summary += " toggles_n=" + std::to_string(m_toggles == nullptr ? 0 : m_toggles->exchange(0, std::memory_order_relaxed));
	summary += " cola_last=" + std::to_string(m_queueSizeLast.exchange(0, std::memory_order_relaxed));
	summary += " cola_max=" + std::to_string(m_queueSizeMax.exchange(0, std::memory_order_relaxed));
	micPrev.accepted = acceptedNow;
	micPrev.consumed = consumedNow;
	return summary;
}

} // namespace twinpad_mic_channel
