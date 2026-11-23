#include "Cafe/Twinpad/Client/TwinpadClient.h"
#include "Cafe/Twinpad/Video/TwinpadH264.h"
#include "Cafe/Twinpad/Video/TwinpadYuvConvert.h"

#include "Common/precompiled.h"

#include <wels/codec_api.h>

#include <atomic>
#include <chrono>
#include <algorithm>
#include <vector>

// ---------------------------------------------------------------------------
// TU del encoder del cliente Twinpad: ResetVideoEncoder (ambos backends) y
// EncodeGamePadFrame. Cadencia real CR-030 (metadata de rate control),
// conversion RGB->YUV CR-043 (modulo twinpad_yuv) y caminos hardware CR-039/
// CR-041/OpenH264 (modulo twinpad_h264); el codigo del monolito original.
// ---------------------------------------------------------------------------

namespace twinpad_cadence
{
uint16_t EstimateFps(const std::deque<uint64_t>& stampsUs, uint16_t fallbackFps)
{
	if (stampsUs.size() < kMinSamples)
		return fallbackFps;
	std::vector<uint64_t> intervals;
	intervals.reserve(stampsUs.size() - 1);
	for (size_t index = 1; index < stampsUs.size(); ++index)
		intervals.push_back(stampsUs[index] - stampsUs[index - 1]);
	std::sort(intervals.begin(), intervals.end());
	const uint64_t medianUs = intervals[intervals.size() / 2];
	if (medianUs == 0)
		return fallbackFps;
	return static_cast<uint16_t>(std::clamp<uint64_t>(1000000 / medianUs, 5, 120));
}

bool CadenceOff(uint32_t encoderFps, uint16_t cadenceFps)
{
	if (encoderFps == 0)
		return false;
	return cadenceFps * 4 > encoderFps * 5 || cadenceFps * 5 < encoderFps * 4; // >25%
}

bool DriftNeedsRecreate(uint32_t encoderFps, uint16_t cadenceFps, uint32_t driftFrames,
	uint64_t sinceRecreateUs)
{
	if (driftFrames < kDriftFrames || sinceRecreateUs < kRecreateCooldownUs)
		return false;
	return CadenceOff(encoderFps, cadenceFps);
}
}

bool TwinpadClient::IsHardwareEncoderActive()
{
	std::scoped_lock encoderLock(m_encoderMutex);
	return m_encoderHardwareActive;
}

void TwinpadClient::ResetVideoEncoder()
{
	std::scoped_lock encoderLock(m_encoderMutex);
	const bool hardwareRequested = m_hardwareAccelerationEnabled.load();
	if (m_hardwareVideoEncoder)
		m_hardwareVideoEncoder.reset();
	if (m_videoEncoder)
	{
		auto* encoder = static_cast<ISVCEncoder*>(m_videoEncoder);
		encoder->Uninitialize();
		WelsDestroySVCEncoder(encoder);
		m_videoEncoder = nullptr;
	}
	m_encoderWidth = 0;
	m_encoderHeight = 0;
	m_encoderBitrateKbps = 0;
	m_encoderNominalFps = 0;
	m_encoderHardwareActive = false;
	m_encoderHardwarePreference = hardwareRequested;
	m_encoderFrameIndex = 0;
	m_forceNextIdr = true;
	m_lastIdrTimestampUs = 0;
	m_encoderEpochUs = 0;
}

bool TwinpadClient::EncodeGamePadFrame(
	const std::vector<uint8_t>& rgb,
	uint16_t width,
	uint16_t height,
	TwinpadEncodedFrame& output)
{
	const auto pipelineStarted = std::chrono::steady_clock::now();
	if (!width || !height || (width & 1) || (height & 1)
		|| rgb.size() != static_cast<size_t>(width) * height * 3)
		return false;

	const uint32_t bitrateKbps = std::clamp(m_captureBitrate.load(), 1000u, 30000u); // CR-027
	const uint64_t nowUs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	std::scoped_lock lock(m_encoderMutex);
	// CR-030 (bitrate): cadencia REAL de emisión para el rate-control. El
	// presupuesto por frame es target/cadencia; con un nominal fijo de 60 la
	// emisión a N fps reales acota el stream a N/60 del bitrate objetivo.
	m_encodeStampsUs.push_back(nowUs);
	if (m_encodeStampsUs.size() > kCadenceWindowFrames)
		m_encodeStampsUs.erase(m_encodeStampsUs.begin());
	// CR-105: mediana de los intervalos, no (n-1)/span. Un tiron aislado de
	// 66 ms en una ventana joven daba 15 fps y ese valor, al ser metadata del
	// rate-control, duplicaba el bitrate hasta la siguiente recreacion.
	const uint16_t fallbackFps = m_encoderNominalFps != 0
		? static_cast<uint16_t>(m_encoderNominalFps) : static_cast<uint16_t>(kEncoderNominalFps);
	const uint16_t cadenceFps = twinpad_cadence::EstimateFps(m_encodeStampsUs, fallbackFps);
	const bool encoderExists = m_videoEncoder || m_hardwareVideoEncoder;
	m_cadenceDriftFrames = encoderExists && twinpad_cadence::CadenceOff(m_encoderNominalFps, cadenceFps)
		? m_cadenceDriftFrames + 1 : 0;
	const bool cadenceStale = twinpad_cadence::DriftNeedsRecreate(m_encoderNominalFps, cadenceFps,
		m_cadenceDriftFrames, nowUs - m_lastEncoderRecreateUs);
	const uint8_t fps = static_cast<uint8_t>(std::min<uint64_t>(cadenceFps, 120u)); // metadata del rate-control; sin pacing (CR-029)
	const bool hardwareRequested = m_hardwareAccelerationEnabled.load();
	// CR-036: la cadencia medida no reinicia el encoder; CR-105 la vuelve a
	// tener en cuenta solo cuando deriva de forma sostenida (cadenceStale).
	const bool encoderConfigChanged = m_encoderWidth != width || m_encoderHeight != height
		|| m_encoderBitrateKbps != bitrateKbps;
	const bool hardwarePreferenceChanged = hardwareRequested != m_encoderHardwarePreference;
	if (!encoderExists || encoderConfigChanged || hardwarePreferenceChanged || cadenceStale)
	{
		// TEMPORAL CR-065-diag: razon exacta de la recreacion del encoder.
		cemuLog_log(LogType::Force,
			"[TwinpadVideo] encoder recreate: exists={} dim_changed={} hw_pref_changed={} cadence_stale={} cadence={} ({}x{} kb={})",
			encoderExists ? 1 : 0, encoderConfigChanged ? 1 : 0, hardwarePreferenceChanged ? 1 : 0,
			cadenceStale ? 1 : 0, cadenceFps, width, height, bitrateKbps);
		if (m_hardwareVideoEncoder)
			m_hardwareVideoEncoder.reset();
		if (m_videoEncoder)
		{
			auto* oldEncoder = static_cast<ISVCEncoder*>(m_videoEncoder);
			oldEncoder->Uninitialize();
			WelsDestroySVCEncoder(oldEncoder);
			m_videoEncoder = nullptr;
		}
		m_encoderHardwareActive = false;
		m_encoderHardwarePreference = hardwareRequested;
		if (hardwareRequested)
		{
			auto hardwareEncoder = twinpad_h264::TwinpadHardwareEncoder::TryCreate(width, height, fps, bitrateKbps);
			if (hardwareEncoder)
			{
				cemuLog_log(LogType::Force, "[TwinpadVideo] encoder=hardware backend={}", hardwareEncoder->GetName());
				m_hardwareVideoEncoder = std::move(hardwareEncoder);
				m_encoderHardwareActive = true;
			}
			else
			{
				cemuLog_log(LogType::Force, "[TwinpadVideo] hardware encoder unavailable; falling back to OpenH264 CPU");
			}
		}
		if (!m_encoderHardwareActive)
		{
			m_videoEncoder = twinpad_h264::CreateOpenH264Encoder(width, height, fps, bitrateKbps);
			if (!m_videoEncoder)
				return false;
			cemuLog_log(LogType::Force, "[TwinpadVideo] encoder=OpenH264 CPU");
		}
		m_encoderWidth = width;
		m_encoderHeight = height;
		m_encoderBitrateKbps = bitrateKbps;
		m_encoderNominalFps = fps;
		m_encoderFrameIndex = 0;
		m_forceNextIdr = true;
		m_lastIdrTimestampUs = 0;
		m_encoderEpochUs = nowUs;
		m_lastEncoderRecreateUs = nowUs;
		m_cadenceDriftFrames = 0;
	}

	// CR-043: conversión RGB8→YUV acelerada (SSSE3 con fallback escalar
	// idéntico bit a bit). Encode hardware consume el buffer NV12 directo;
	// OpenH264 sigue recibiendo I420.
	const auto yuvStarted = std::chrono::steady_clock::now();
	const size_t ySize = static_cast<size_t>(width) * height;
	const bool hardwareAttempted = m_encoderHardwareActive;
	twinpad_yuv::ConvertRgbToYuv(rgb.data(), width, height, hardwareAttempted, m_yuvBuffer);
	RecordVideoMetric(TwinpadVideoMetricStage::I420_CONVERT,
		std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - yuvStarted).count());
	uint8_t* const yPlane = m_yuvBuffer.data();
	uint8_t* const uPlane = yPlane + ySize;
	uint8_t* const vPlane = uPlane + ySize / 4;

	output.annexB.clear();
	bool forceIdr = m_forceNextIdr || nowUs - m_lastIdrTimestampUs >= kIdrIntervalMs * 1000;
	// TEMPORAL CR-065-diag: estado IDR visible en el log de Cemu cada 2 s.
	// El dropeo de una AU forzada sin IDR no consume fn ni frame: sin esta
	// línea la sequía de IDR es invisible salvo por la ausencia en el stream.
	static std::atomic<uint64_t> sIdrDiagNextMs{0};
	const uint64_t idrDiagMs = nowUs / 1000;
	uint64_t idrDiagNext = sIdrDiagNextMs.load(std::memory_order_relaxed);
	if (idrDiagMs >= idrDiagNext
		&& sIdrDiagNextMs.compare_exchange_weak(idrDiagNext, idrDiagMs + 2000,
			std::memory_order_relaxed, std::memory_order_relaxed))
	{
		cemuLog_log(LogType::Force,
			"[TwinpadVideo] idr_diag want={} force_next={} since_last_idr_ms={} hw_active={} index={} cadence={} enc_fps={}",
			forceIdr ? 1 : 0,
			m_forceNextIdr.load(std::memory_order_relaxed) ? 1 : 0,
			m_lastIdrTimestampUs != 0 ? (nowUs - m_lastIdrTimestampUs) / 1000 : -1,
			m_encoderHardwareActive ? 1 : 0,
			m_encoderFrameIndex,
			cadenceFps,
			m_encoderNominalFps);
	}
	auto fallbackToOpenH264 = [&]() {
		m_hardwareVideoEncoder.reset();
		m_encoderHardwareActive = false;
		m_videoEncoder = twinpad_h264::CreateOpenH264Encoder(width, height, fps, bitrateKbps);
		if (!m_videoEncoder)
			return false;
		output.annexB.clear();
		m_forceNextIdr = true;
		m_lastIdrTimestampUs = 0;
		m_encoderEpochUs = nowUs;
		forceIdr = true;
		cemuLog_log(LogType::Force, "[TwinpadVideo] hardware encoder violated the H.264 contract; falling back to OpenH264 CPU");
		return true;
	};
	if (m_encoderHardwareActive)
	{
		const auto hardwareStarted = std::chrono::steady_clock::now();
		auto* encoder = m_hardwareVideoEncoder.get();
		// CR-043: se entrega el buffer NV12 ya consolidado (Y + UV intercalado).
		const bool hardwareEncoded = encoder->Encode(yPlane, uPlane, width, height,
			static_cast<int64_t>(m_encoderFrameIndex), forceIdr, output.annexB);
		// VAAPI con pipeline interno devuelve EAGAIN: no es fallo, la AU se emite
		// en ticks posteriores (se avisa solo la primera vez y se conserva el backend).
		if (!hardwareEncoded && encoder->HasBufferedOutput())
		{
			static std::atomic_bool warnedBufferedOnce{false};
			if (!warnedBufferedOnce.exchange(true))
				cemuLog_log(LogType::Force, "[TwinpadVideo] hardware backend {} buffers frames (EAGAIN); AUs will arrive on later ticks", encoder->GetName());
			return false;
		}
		if (!hardwareEncoded)
			cemuLog_log(LogType::Force, "[TwinpadVideo] hardware backend {} did not produce an output AU on frame {} (idr_forced={})",
				encoder->GetName(), m_encoderFrameIndex, forceIdr ? 1 : 0);
		if (!hardwareEncoded)
		{
			if (!fallbackToOpenH264())
				return false;
		}
		else
		{
			RecordVideoMetric(TwinpadVideoMetricStage::HARDWARE_ENCODER,
				std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - hardwareStarted).count());
			output.idr = twinpad_h264::ContainsIdrNal(output.annexB);
		}
		if (!hardwareEncoded || !m_encoderHardwareActive)
		{
			// CR-043: si el frame entrante era NV12 (hardwareAttempted) y caímos a
			// OpenH264, reconvertir a I420 en el mismo buffer antes del encode CPU.
			if (hardwareAttempted && !m_encoderHardwareActive)
				twinpad_yuv::ConvertRgbToYuv(rgb.data(), width, height, false, m_yuvBuffer);
			SSourcePicture picture{};
			picture.iColorFormat = videoFormatI420;
			picture.iPicWidth = width;
			picture.iPicHeight = height;
			picture.iStride[0] = width;
			picture.iStride[1] = width / 2;
			picture.iStride[2] = width / 2;
			picture.pData[0] = yPlane;
			picture.pData[1] = uPlane;
			picture.pData[2] = vPlane;
			picture.uiTimeStamp = static_cast<long long>((nowUs - m_encoderEpochUs) / 1000); // CR-029: tiempo real
			auto* cpuEncoder = static_cast<ISVCEncoder*>(m_videoEncoder);
			if (forceIdr && cpuEncoder->ForceIntraFrame(true) != 0)
				return false;
			SFrameBSInfo info{};
			const auto openH264Started = std::chrono::steady_clock::now();
			const int encodeResult = cpuEncoder->EncodeFrame(&picture, &info);
			RecordVideoMetric(TwinpadVideoMetricStage::OPENH264,
				std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - openH264Started).count());
			if (encodeResult != 0 || info.eFrameType == videoFrameTypeSkip || info.eFrameType == videoFrameTypeInvalid)
				return false;
			for (int layerIndex = 0; layerIndex < info.iLayerNum; ++layerIndex)
			{
				const auto& layer = info.sLayerInfo[layerIndex];
				const uint8_t* nalData = layer.pBsBuf;
				for (int nalIndex = 0; nalIndex < layer.iNalCount; ++nalIndex)
				{
					const int nalLength = layer.pNalLengthInByte[nalIndex];
					if (nalLength <= 0 || output.annexB.size() + static_cast<size_t>(nalLength) > 4 * 1024 * 1024)
						return false;
					output.annexB.insert(output.annexB.end(), nalData, nalData + nalLength);
					nalData += nalLength;
				}
			}
			if (output.annexB.empty())
				return false;
			output.idr = info.eFrameType == videoFrameTypeIDR;
		}
	}
	else
	{
		SSourcePicture picture{};
		picture.iColorFormat = videoFormatI420;
		picture.iPicWidth = width;
		picture.iPicHeight = height;
		picture.iStride[0] = width;
		picture.iStride[1] = width / 2;
		picture.iStride[2] = width / 2;
		picture.pData[0] = yPlane;
		picture.pData[1] = uPlane;
		picture.pData[2] = vPlane;
		picture.uiTimeStamp = static_cast<long long>((nowUs - m_encoderEpochUs) / 1000); // CR-029: tiempo real
		auto* encoder = static_cast<ISVCEncoder*>(m_videoEncoder);
		if (forceIdr && encoder->ForceIntraFrame(true) != 0)
			return false;
		SFrameBSInfo info{};
		const auto openH264Started = std::chrono::steady_clock::now();
		const int encodeResult = encoder->EncodeFrame(&picture, &info);
		RecordVideoMetric(TwinpadVideoMetricStage::OPENH264,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - openH264Started).count());
		if (encodeResult != 0 || info.eFrameType == videoFrameTypeSkip || info.eFrameType == videoFrameTypeInvalid)
			return false;
		for (int layerIndex = 0; layerIndex < info.iLayerNum; ++layerIndex)
		{
			const auto& layer = info.sLayerInfo[layerIndex];
			const uint8_t* nalData = layer.pBsBuf;
			for (int nalIndex = 0; nalIndex < layer.iNalCount; ++nalIndex)
			{
				const int nalLength = layer.pNalLengthInByte[nalIndex];
				if (nalLength <= 0 || output.annexB.size() + static_cast<size_t>(nalLength) > 4 * 1024 * 1024)
					return false;
				output.annexB.insert(output.annexB.end(), nalData, nalData + nalLength);
				nalData += nalLength;
			}
		}
		if (output.annexB.empty())
			return false;
		output.idr = info.eFrameType == videoFrameTypeIDR;
	}
	output.timestampUs = nowUs >= m_encoderEpochUs ? nowUs - m_encoderEpochUs : 0;
	if (forceIdr && (!output.idr || !twinpad_h264::ContainsH264Nal(output.annexB.data(), output.annexB.size(), twinpad_h264::kH264SpsNalType)
		|| !twinpad_h264::ContainsH264Nal(output.annexB.data(), output.annexB.size(), twinpad_h264::kH264PpsNalType)
		|| !twinpad_h264::HasH264ContractProfile(output.annexB)))
	{
		// TEMPORAL CR-065-diag: cada dropeo por forceIdr fallido visible.
		cemuLog_log(LogType::Force,
			"[TwinpadVideo] idr_drop index={} idr_flag={} contains_idr={} sps={} pps={} contract={}",
			m_encoderFrameIndex,
			output.idr ? 1 : 0,
			twinpad_h264::ContainsIdrNal(output.annexB) ? 1 : 0,
			twinpad_h264::ContainsH264Nal(output.annexB.data(), output.annexB.size(), twinpad_h264::kH264SpsNalType) ? 1 : 0,
			twinpad_h264::ContainsH264Nal(output.annexB.data(), output.annexB.size(), twinpad_h264::kH264PpsNalType) ? 1 : 0,
			twinpad_h264::HasH264ContractProfile(output.annexB) ? 1 : 0);
		return false;
	}
	if (output.idr)
	{
		m_lastIdrTimestampUs = nowUs;
		m_forceNextIdr = false;
	}
	++m_encoderFrameIndex;
	RecordVideoMetric(TwinpadVideoMetricStage::ENCODE_PIPELINE,
		std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - pipelineStarted).count());
	return true;
}
