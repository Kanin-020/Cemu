#include "Cafe/Twinpad/Client/TwinpadClient.h"
#include "Cafe/Twinpad/Video/TwinpadYuvConvert.h"

#include "Common/precompiled.h"

#include <condition_variable>
#include <deque>
#include <filesystem>
#include <fstream>
#include <unordered_set>

#if BOOST_OS_WINDOWS
#include <winsock2.h>
using PipelineSocketHandle = SOCKET;
static void ClosePipelineSocket(PipelineSocketHandle socket) { if (socket >= 0) closesocket(socket); }
#else
#include <sys/socket.h>
#include <unistd.h>
using PipelineSocketHandle = int;
static void ClosePipelineSocket(PipelineSocketHandle socket) { if (socket >= 0) close(socket); }
#endif

// ---------------------------------------------------------------------------
// TU del pipeline de salida del cliente Twinpad: ingreso de frames/audio,
// publicaciones (rumble/camera_request) y los hilos FrameLoop, VideoSendLoop
// y AudioLoop del monolito original (transplante verbatim; solo cambia la
// delegacion de robotica a twinpad_yuv (SrgbToRgb/ScaleRgb) y el encoder.
// ---------------------------------------------------------------------------

namespace
{

// CR-050: volcado del primer RGB convertido de cada clave (formato,w,h) a
// TWINPAD_CAPTURE_DUMP_DIR (max 6 dumps por sesion); evidencia de
// canal/gamma/pitch sin tocar producción.
void TwinpadCaptureDump(const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height, TwinpadPixelFormat format)
{
	static std::unordered_set<uint64_t> dumpedKeys;
	static uint32_t dumpCount = 0;
	if (dumpCount >= 6)
		return;
	const char* dumpDir = std::getenv("TWINPAD_CAPTURE_DUMP_DIR");
	if (!dumpDir)
		return;
	const uint64_t key = (static_cast<uint64_t>(width) << 24) | (static_cast<uint64_t>(height) << 8)
		| (static_cast<uint64_t>(static_cast<uint8_t>(format)));
	if (!dumpedKeys.insert(key).second)
		return;
	const std::filesystem::path path = std::filesystem::path(dumpDir)
		/ (std::string("twinpad-frame-") + TwinpadFormatName(format) + "-" + std::to_string(width) + "x" + std::to_string(height) + ".ppm");
	std::error_code errorCode;
	std::filesystem::create_directories(std::filesystem::path(dumpDir), errorCode);
	std::ofstream file;
	file.open(path, std::ios::binary);
	if (!file.is_open())
	{
		cemuLog_log(LogType::Force, "[TwinpadDiag] dump PPM no escribible: {}", path.string());
		return;
	}
	file << "P6\n" << width << " " << height << "\n255\n";
	file.write(reinterpret_cast<const char*>(rgb.data()), static_cast<std::streamsize>(rgb.size()));
}

} // namespace

void TwinpadClient::PublishGamePadFrame(
	std::vector<uint8_t>&& pixels,
	uint16_t width,
	uint16_t height,
	TwinpadPixelFormat format)
{
	const size_t bytesPerPixel = format == TwinpadPixelFormat::Rgb8 || format == TwinpadPixelFormat::Rgb8Srgb ? 3 : 4;
	if (!m_connected.load() || !m_captureGamePad.load() || !width || !height
		|| pixels.size() != static_cast<size_t>(width) * height * bytesPerPixel)
		return;
	// CR-029: sin pacing por fps; cada frame capturado entra en cola.
	{
		std::scoped_lock lock(m_frameQueueMutex);
		m_pendingFrame = std::move(pixels);
		m_pendingFrameWidth = width;
		m_pendingFrameHeight = height;
		m_pendingFrameFormat = format;
		m_hasPendingFrame = true;
	}
	RecordVideoEvent(TwinpadVideoMetricEvent::FRAME_QUEUED);
	m_frameQueueChanged.notify_one();
}

void TwinpadClient::PublishAudioBlock(const int16_t* samples, size_t samplesPerChannel, uint8_t channels)
{
	if (!m_connected.load() || !samples || samplesPerChannel == 0 || channels == 0)
		return;
	std::vector<int16_t> block;
	block.reserve(samplesPerChannel * 2);
	for (size_t sample = 0; sample < samplesPerChannel; ++sample)
	{
		const int16_t left = samples[sample * channels];
		const int16_t right = channels > 1 ? samples[sample * channels + 1] : left;
		block.push_back(left);
		block.push_back(right);
	}
	{
		std::scoped_lock lock(m_audioQueueMutex);
		constexpr size_t kAudioQueueCapacity = 8;
		if (m_audioQueue.size() == kAudioQueueCapacity)
			m_audioQueue.pop_front();
		m_audioQueue.push_back(std::move(block));
	}
	m_audioQueueChanged.notify_one();
}

void TwinpadClient::FrameLoop()
{
	TwinpadPixelFormat lastDiagFormat = static_cast<TwinpadPixelFormat>(0xFF);
	while (m_running.load())
	{
		FlushVideoMetricsIfDue();
		std::vector<uint8_t> frameData;
		uint16_t width = 0;
		uint16_t height = 0;
		TwinpadPixelFormat format = TwinpadPixelFormat::Rgb8;
		{
			std::unique_lock lock(m_frameQueueMutex);

			const bool gotFrame = m_frameQueueChanged.wait_for(lock, std::chrono::milliseconds(kFrameRepeatIntervalMs),
				[this] { return !m_running.load() || m_hasPendingFrame; });
			if (!m_running.load())
				break;
			if (!m_connected.load())
			{
				m_hasPendingFrame = false;
				m_lastFrameRgb.clear();
				continue;
			}
			if (gotFrame)
			{
				frameData = std::move(m_pendingFrame);
				width = m_pendingFrameWidth;
				height = m_pendingFrameHeight;
				format = m_pendingFrameFormat;
				m_hasPendingFrame = false;
				// CR-075: resolución de entrada del pipeline (fuente del juego,
				// antes de la etapa SCALE); solo frame fresco la actualiza.
				m_lastSourceWidth = width;
				m_lastSourceHeight = height;
			}
			else
			{
				// CR-036: sin frame nuevo en el intervalo, repetir el último ya escalado (RGB8).
				if (m_lastFrameRgb.empty())
					continue;
				frameData = m_lastFrameRgb;
				width = m_lastFrameWidth;
				height = m_lastFrameHeight;
				format = TwinpadPixelFormat::Rgb8;
			}
		}

		const auto pixelConvertStarted = std::chrono::steady_clock::now();
		// CR-050: cada cambio de formato del gamepad se loguea con su tamaño; el
		// camino CR-036 (directo) y el CR-040 (blit GPU) quedan identificables
		// por juego en el log (la línea de CR-040 solo se emite una vez por sesión).
		if (format != lastDiagFormat)
		{
			lastDiagFormat = format;
			cemuLog_log(LogType::Force, "[TwinpadDiag] frame del gamepad: fmt={} ({}x{})", TwinpadFormatName(format), width, height);
			// CR-050: muestra del RGB convertido (lo que come el encoder) en el
			// cambio de formato: promedio/banda por canal y 3 píxeles de la fila
			// media. Canal intercambiado, gamma faltante o sobrante y pitch se
			// distinguen leyendo estos números (y comparándolos con la vista DRC
			// de la ventana de Cemu).
			if (static_cast<size_t>(width) * height * 3 == frameData.size())
			{
				uint64_t sumR = 0, sumG = 0, sumB = 0;
				uint8_t minR = 255, minG = 255, minB = 255, maxR = 0, maxG = 0, maxB = 0;
				const size_t pixelCount = static_cast<size_t>(width) * height;
				for (size_t pixel = 0; pixel < pixelCount; ++pixel)
				{
					const uint8_t r = frameData[pixel * 3], g = frameData[pixel * 3 + 1], b = frameData[pixel * 3 + 2];
					sumR += r; sumG += g; sumB += b;
					minR = std::min(minR, r); minG = std::min(minG, g); minB = std::min(minB, b);
					maxR = std::max(maxR, r); maxG = std::max(maxG, g); maxB = std::max(maxB, b);
				}
				cemuLog_log(LogType::Force,
					"[TwinpadDiag] fmt={} stats avg=({:.2f},{:.2f},{:.2f}) r=({},{}) g=({},{}) b=({},{})",
					TwinpadFormatName(format),
					static_cast<double>(sumR) / pixelCount, static_cast<double>(sumG) / pixelCount,
					static_cast<double>(sumB) / pixelCount, minR, maxR, minG, maxG, minB, maxB);
				const size_t row = static_cast<size_t>(height / 2);
				const auto samplePixel = [&](uint32_t x) {
					const size_t index = (row * width + x) * 3;
					return fmt::format("({},{})=({},{},{})", x, row,
						static_cast<uint32_t>(frameData[index]), static_cast<uint32_t>(frameData[index + 1]),
						static_cast<uint32_t>(frameData[index + 2]));
				};
				cemuLog_log(LogType::Force, "[TwinpadDiag] fila-mid: {} {} {}",
					samplePixel(width / 4), samplePixel(width / 2), samplePixel(width * 3 / 4));
			}
		}
		if (format == TwinpadPixelFormat::Rgb8Srgb)
		{
			// CR-036: conversión con LUT del módulo yuv (bit exacto con la
			// tabla de 256 entradas usada por el monolito).
			std::transform(frameData.begin(), frameData.end(), frameData.begin(), twinpad_yuv::SrgbToRgb);
		}
		else if (format == TwinpadPixelFormat::Rgba8 || format == TwinpadPixelFormat::Rgba8Srgb
			|| format == TwinpadPixelFormat::Bgra8 || format == TwinpadPixelFormat::Bgra8Srgb)
		{
			// CR-036: BGRA del DRC (p. ej. Wind Waker HD) requiere el intercambio
			// de los canales B y R para producir RGB; sRGB aplica gamma igual.
			const bool swapBr = format == TwinpadPixelFormat::Bgra8 || format == TwinpadPixelFormat::Bgra8Srgb;
			const bool applySrgb = format == TwinpadPixelFormat::Rgba8Srgb || format == TwinpadPixelFormat::Bgra8Srgb;
			const size_t pixelCount = static_cast<size_t>(width) * height;
			std::vector<uint8_t> rgb(pixelCount * 3);
			for (size_t pixel = 0; pixel < pixelCount; ++pixel)
			{
				const size_t source = pixel * 4;
				const size_t destination = pixel * 3;
				for (size_t channel = 0; channel < 3; ++channel)
				{
					const size_t sourceChannel = swapBr && channel == 0 ? 2 : swapBr && channel == 2 ? 0 : channel;
					const uint8_t value = frameData[source + sourceChannel];
					rgb[destination + channel] = applySrgb ? twinpad_yuv::SrgbToRgb(value) : value;
				}
			}
			frameData = std::move(rgb);
		}
		RecordVideoMetric(TwinpadVideoMetricStage::PIXEL_CONVERT,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - pixelConvertStarted).count());
		// CR-050: vuelca el RGB convertido (PPM P6) del primer frame por clave
		// (formato,w,h) con TWINPAD_CAPTURE_DUMP_DIR definida; evidencia para
		// el fix de color (canal/gamma/pitch) sin tocar la ruta de producción.
		TwinpadCaptureDump(frameData, width, height, format);
		// CR-030/CR-074: aplicar la resolución de emisión del GamePad (DRC) en
		// ambas direcciones (upscale permitido); «Nativa» (0/0) no escala.
		const uint32_t targetWidth = m_captureWidth.load(std::memory_order_relaxed);
		const uint32_t targetHeight = m_captureHeight.load(std::memory_order_relaxed);
		if (targetWidth > 0 && targetHeight > 0
			&& (targetWidth != static_cast<uint32_t>(width)
				|| targetHeight != static_cast<uint32_t>(height)))
		{
			const auto scaleStarted = std::chrono::steady_clock::now();
			frameData = twinpad_yuv::ScaleRgb(frameData, width, height, static_cast<uint16_t>(targetWidth), static_cast<uint16_t>(targetHeight));
			width = static_cast<uint16_t>(targetWidth);
			height = static_cast<uint16_t>(targetHeight);
			RecordVideoMetric(TwinpadVideoMetricStage::SCALE,
				std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - scaleStarted).count());
		}
		m_lastFrameRgb = frameData;
		m_lastFrameWidth = width;
		m_lastFrameHeight = height;
		TwinpadEncodedFrame encodedFrame;
		if (!EncodeGamePadFrame(frameData, width, height, encodedFrame))
			continue;
		RecordVideoEvent(TwinpadVideoMetricEvent::FRAME_ENCODED);
		// CR-054: el AU va a la cola sin base64 ni JSON; la serialización
		// (binaria o NDJSON de fallback) corre en el hilo de envío.
		TwinpadQueuedAu queued;
		queued.annexB = std::move(encodedFrame.annexB);
		queued.gfn = m_frameNumber.fetch_add(1);
		queued.timestampUs = encodedFrame.timestampUs;
		queued.width = width;
		queued.height = height;
		queued.idr = encodedFrame.idr;
		// CR-048: la línea pasa al hilo de envío; el FrameLoop ya no espera
		// al TCP (la backpressure dejaba la cadencia del encode a merced del
		// socket). Drop-old en ráfagas: la imagen conserva la firma más fresca.
		{
			std::scoped_lock lock(m_videoSendQueueMutex);
			if (m_videoSendQueue.size() == kVideoSendQueueMaxLines)
				m_videoSendQueue.pop_front();
			m_videoSendQueue.push_back(std::move(queued));
		}
		m_videoSendQueueChanged.notify_one();
	}
}

void TwinpadClient::VideoSendLoop()
{
	while (m_running.load())
	{
		TwinpadQueuedAu queued;
		{
			std::unique_lock lock(m_videoSendQueueMutex);
			m_videoSendQueueChanged.wait(lock, [this] { return !m_running.load() || !m_videoSendQueue.empty(); });
			if (!m_running.load())
				break;
			queued = std::move(m_videoSendQueue.front());
			m_videoSendQueue.pop_front();
		}
		// CR-054: camino binario preferente (sin base64/JSON). Si el canal
		// media no está vivo, el fallback NDJSON conserva el flujo viejo.
		bool sent = false;
		if (m_mediaConnected.load() && queued.ndjsonLine.empty())
		{
			const auto ipcSendStarted = std::chrono::steady_clock::now();
			std::scoped_lock lock(m_mediaSocketMutex);
			sent = m_mediaSocket >= 0 && SendMediaFrame(queued);
			if (!sent)
			{
				// Canal muerto: volver al NDJSON en los frames siguientes.
				m_mediaConnected.store(false);
				if (m_mediaSocket >= 0)
				{
					ClosePipelineSocket(static_cast<PipelineSocketHandle>(m_mediaSocket));
					m_mediaSocket = -1;
				}
				cemuLog_log(LogType::Force, "[Twinpad] canal media binario cayó; fallback a NDJSON (CR-054).");
			}
			else
			{
				RecordVideoMetric(TwinpadVideoMetricStage::IPC_SEND,
					std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - ipcSendStarted).count());
				RecordVideoEvent(TwinpadVideoMetricEvent::FRAME_SENT);
				m_lastFrameSentMs.store(static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count()));
				continue;
			}
		}
		// Fallback NDJSON: serializa aquí solo cuando el binario no está.
		// CR-053: los ítems NDJSON puros (`video_stage_stats`) salen tal
		// cual por el canal primario; solo los AU binarios se envuelven en
		// `game_frame`. En CR-054 el fallback re-serializaba el ítem de
		// stats como frame con `data`:"" y el HUD nunca recibía la línea.
		const auto ipcSendStarted = std::chrono::steady_clock::now();
		std::string line;
		if (!queued.ndjsonLine.empty())
		{
			line = std::move(queued.ndjsonLine);
		}
		else
		{
			const auto base64JsonStarted = std::chrono::steady_clock::now();
			const std::string encodedData = Base64Url(queued.annexB.data(), queued.annexB.size());
			line.reserve(encodedData.size() + 192);
			line = "{\"type\":\"game_frame\",\"gfn\":" + std::to_string(queued.gfn)
				+ ",\"w\":" + std::to_string(queued.width) + ",\"h\":" + std::to_string(queued.height)
				+ ",\"fmt\":\"h264\",\"ts_capture_us\":" + std::to_string(queued.timestampUs)
				+ ",\"idr\":" + (queued.idr ? "true" : "false")
				+ ",\"data\":\"" + encodedData + "\"}";
			RecordVideoMetric(TwinpadVideoMetricStage::BASE64_JSON,
				std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - base64JsonStarted).count());
		}
		std::scoped_lock lock(m_primarySocketMutex);
		if (!m_connected.load() || m_primarySocket < 0)
		{
			// Desconectado: no acumular backlog sobre la reconexión.
			std::scoped_lock queueLock(m_videoSendQueueMutex);
			m_videoSendQueue.clear();
			continue;
		}
		if (!SendLine(m_primarySocket, line))
		{
			m_connected.store(false);
			std::scoped_lock queueLock(m_videoSendQueueMutex);
			m_videoSendQueue.clear();
			continue;
		}
		RecordVideoMetric(TwinpadVideoMetricStage::IPC_SEND,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - ipcSendStarted).count());
		RecordVideoEvent(TwinpadVideoMetricEvent::FRAME_SENT);
		m_lastFrameSentMs.store(static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
			std::chrono::steady_clock::now().time_since_epoch()).count()));
	}
}

void TwinpadClient::AudioLoop()
{
	while (m_running.load())
	{
		std::vector<int16_t> block;
		{
			std::unique_lock lock(m_audioQueueMutex);
			m_audioQueueChanged.wait(lock, [this] { return !m_running.load() || !m_audioQueue.empty(); });
			if (!m_running.load())
				break;
			block = std::move(m_audioQueue.front());
			m_audioQueue.pop_front();
		}
		const auto* bytes = reinterpret_cast<const uint8_t*>(block.data());
		const size_t byteCount = block.size() * sizeof(int16_t);
		constexpr size_t kMaxPcmChunkBytes = 16 * 1024;
		for (size_t offset = 0; offset < byteCount; offset += kMaxPcmChunkBytes)
		{
			const size_t amount = std::min(kMaxPcmChunkBytes, byteCount - offset);
			const uint64_t seq = m_audioSequence.fetch_add(1);
			const std::string line = "{\"type\":\"game_audio_chunk\",\"game_id\":\"cemu\",\"seq\":"
				+ std::to_string(seq) + ",\"ts\":"
				+ std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(
					std::chrono::steady_clock::now().time_since_epoch()).count())
				+ ",\"data\":\"" + Base64Url(bytes + offset, amount) + "\"}";
			std::scoped_lock lock(m_audioSocketMutex);
			if (m_audioSocket < 0 || !SendLine(m_audioSocket, line))
			{
				m_connected.store(false);
				if (m_audioSocket >= 0)
					ClosePipelineSocket(static_cast<PipelineSocketHandle>(m_audioSocket));
				m_audioSocket = -1;
				break;
			}
		}
	}
}
