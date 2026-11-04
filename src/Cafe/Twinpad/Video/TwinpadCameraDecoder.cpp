#include "Common/precompiled.h"
#include "Cafe/Twinpad/Video/TwinpadCameraDecoder.h"

#include <rapidjson/document.h>

#include <atomic>
#include <chrono>
#include <cstring>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavutil/frame.h>
}

namespace
{
int DecodeBase64UrlChar(char c)
{
	if (c >= 'A' && c <= 'Z') return c - 'A';
	if (c >= 'a' && c <= 'z') return c - 'a' + 26;
	if (c >= '0' && c <= '9') return c - '0' + 52;
	if (c == '-') return 62;
	if (c == '_') return 63;
	return -1;
}

// base64url sin relleno (SPEC-006: `data`).
bool DecodeBase64Url(const char* text, size_t size, std::vector<uint8_t>& out)
{
	out.clear();
	if (size % 4 == 1)
		return false;
	uint32_t buffer = 0;
	int bits = 0;
	for (size_t i = 0; i < size; ++i)
	{
		const int value = DecodeBase64UrlChar(text[i]);
		if (value < 0)
			return false;
		buffer = (buffer << 6) | static_cast<uint32_t>(value);
		bits += 6;
		if (bits >= 8)
		{
			bits -= 8;
			out.push_back(static_cast<uint8_t>((buffer >> bits) & 0xff));
		}
	}
	return !out.empty();
}
}

// Diagnóstico CR-034: loguea ≤ 1 Hz el camino de camera_frame; un descarte
// silencioso del decoder es indistinguible desde el socket.
bool CameraLogSlot()
{
	static std::atomic<uint64_t> sLastLogMs{0};
	const auto nowMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	uint64_t previousMs = sLastLogMs.load();
	if (nowMs - previousMs < 1000)
		return false;
	return sLastLogMs.compare_exchange_weak(previousMs, nowMs);
}

struct TwinpadCameraDecoder::Impl
{
	AVCodecContext* context{};
	AVFrame* frame{};
	AVPacket* packet{};

	~Impl()
	{
		av_packet_free(&packet);
		av_frame_free(&frame);
		avcodec_free_context(&context);
	}
};

TwinpadCameraDecoder& TwinpadCameraDecoder::Instance()
{
	static TwinpadCameraDecoder instance;
	return instance;
}

TwinpadCameraDecoder::TwinpadCameraDecoder()
{
	auto impl = std::make_unique<Impl>();
	const AVCodec* codec = avcodec_find_decoder(AV_CODEC_ID_H264);
	if (!codec)
		return;
	impl->context = avcodec_alloc_context3(codec);
	impl->frame = av_frame_alloc();
	impl->packet = av_packet_alloc();
	if (!impl->context || !impl->frame || !impl->packet)
		return;
	if (avcodec_open2(impl->context, codec, nullptr) < 0)
		return;
	m_impl = impl.release();
}

TwinpadCameraDecoder::~TwinpadCameraDecoder()
{
	delete m_impl;
}

bool TwinpadCameraDecoder::DecodeLine(const std::string& line)
{
	if (!m_impl)
	{
		if (CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] camera_frame descartado (motivo: decoder_no_inicializado)");
		return false;
	}
	rapidjson::Document document;
	document.Parse(line.c_str(), line.size());
	if (document.HasParseError() || !document.IsObject())
	{
		if (CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] camera_frame descartado (motivo: json_invalido)");
		return false;
	}
	if (!document.HasMember("type") || !document["type"].IsString()
		|| std::string_view(document["type"].GetString()) != "camera_frame")
	{
		if (CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] camera_frame descartado (motivo: tipo)");
		return false;
	}
	if (!document.HasMember("w") || !document["w"].IsUint() || document["w"].GetUint() != kWidth)
	{
		if (CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] camera_frame descartado (motivo: geometria_w)");
		return false;
	}
	if (!document.HasMember("h") || !document["h"].IsUint() || document["h"].GetUint() != kHeight)
	{
		if (CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] camera_frame descartado (motivo: geometria_h)");
		return false;
	}
	if (!document.HasMember("data") || !document["data"].IsString())
	{
		if (CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] camera_frame descartado (motivo: sin_data)");
		return false;
	}

	std::vector<uint8_t> accessUnit;
	if (!DecodeBase64Url(document["data"].GetString(), document["data"].GetStringLength(), accessUnit))
	{
		if (CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] camera_frame descartado (motivo: base64url_invalido)");
		return false;
	}

	av_packet_unref(m_impl->packet);
	if (av_new_packet(m_impl->packet, static_cast<int>(accessUnit.size())) < 0)
		return false;
	std::memcpy(m_impl->packet->data, accessUnit.data(), accessUnit.size());

	if (avcodec_send_packet(m_impl->context, m_impl->packet) < 0)
	{
		if (CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] camera_frame descartado (motivo: send_rechazado, bytes={})", accessUnit.size());
		return false;
	}

	bool decoded = false;
	while (avcodec_receive_frame(m_impl->context, m_impl->frame) == 0)
	{
		AVFrame* f = m_impl->frame;
		if (f->width != kWidth || f->height != kHeight || f->format != AV_PIX_FMT_YUV420P)
		{
			av_frame_unref(f);
			continue;
		}
		std::vector<uint8_t> nv12;
		const bool converted = ConvertToNv12(
			f->data[0], f->linesize[0], f->data[1], f->data[2], f->linesize[1], kWidth, kHeight, nv12);
		if (converted)
		{
			std::scoped_lock lock(m_mutex);
			m_nv12 = std::move(nv12);
			decoded = true;
		}
		av_frame_unref(f);
	}
	if (decoded)
	{
		// Diagnóstico CR-034: ritmo de decodificación real (~128 a 30 fps ≈ 4 s).
		static std::atomic<uint32_t> sDecodedFrames{0};
		if (sDecodedFrames.fetch_add(1) % 128 == 0 && CameraLogSlot())
			cemuLog_log(LogType::Force, "[TwinpadCamera] frame decode ok (total={})", sDecodedFrames.load());
	}
	return decoded;
}

bool TwinpadCameraDecoder::ConvertToNv12(
	const uint8_t* y, int yStride,
	const uint8_t* u, const uint8_t* v, int uvStride,
	uint16_t width, uint16_t height,
	std::vector<uint8_t>& out)
{
	if (!y || !u || !v || width == 0 || height == 0 || (width % 2) || (height % 2))
		return false;
	const size_t ySize = static_cast<size_t>(width) * height;
	out.assign(ySize + ySize / 2, 0);
	for (int row = 0; row < height; ++row)
		std::memcpy(out.data() + static_cast<size_t>(row) * width, y + row * yStride, width);
	uint8_t* uv = out.data() + ySize;
	for (int row = 0; row < height / 2; ++row)
		for (int column = 0; column < width / 2; ++column)
		{
			uv[static_cast<size_t>(row) * width + column * 2] = u[row * uvStride + column];
			uv[static_cast<size_t>(row) * width + column * 2 + 1] = v[row * uvStride + column];
		}
	return true;
}

size_t TwinpadCameraDecoder::CopyLatestNv12(uint8_t* dst, size_t dstSize)
{
	std::scoped_lock lock(m_mutex);
	if (!dst || m_nv12.empty() || dstSize < m_nv12.size())
		return 0;
	std::memcpy(dst, m_nv12.data(), m_nv12.size());
	return m_nv12.size();
}

size_t TwinpadCameraDecoder::ScaleLatestNv12(uint8_t* dst, size_t dstSize, size_t dstPitch, size_t dstHeight,
	size_t dstVisibleWidth)
{
	std::scoped_lock lock(m_mutex);
	if (!dst || dstPitch == 0 || dstHeight == 0 || (dstPitch % 2) || (dstHeight % 2)
		|| dstVisibleWidth == 0 || dstVisibleWidth > dstPitch || m_nv12.empty())
		return 0;
	// CR-046: NV12 completo = pitch*height (Y) + pitch*height/2 (UV intercalado).
	const size_t want = dstPitch * dstHeight + dstPitch * dstHeight / 2;
	if (dstSize < want)
		return 0;

	// Sin basura: todo el surface queda definido por el frame escalado
	// (el padding de cada fila, pitch-visibleWidth, queda en cero).
	std::memset(dst, 0, dstSize);

	constexpr uint16_t srcW = kWidth;
	constexpr uint16_t srcH = kHeight;
	// Plano Y: escala nearest-neighbor a las columnas visibles [0, dstVisibleWidth).
	for (size_t row = 0; row < dstHeight; ++row)
	{
		const uint8_t* srcRow = m_nv12.data() + (row * srcH / dstHeight) * srcW;
		uint8_t* dstRow = dst + row * dstPitch;
		for (size_t column = 0; column < dstVisibleWidth; ++column)
			dstRow[column] = srcRow[column * (srcW - 1) / (dstVisibleWidth - 1)];
	}

	// Plano UV intercalado (U,V por par de píxeles); mismo pitch de surface.
	uint8_t* dstUV = dst + dstPitch * dstHeight;
	const uint8_t* srcUV = m_nv12.data() + static_cast<size_t>(srcW) * srcH;
	constexpr uint16_t srcUVH = (uint16_t)(srcH / 2);
	const size_t dstUVH = dstHeight / 2;
	const size_t dstPairs = dstVisibleWidth / 2; // pares U,V visibles por fila
	const size_t srcPairs = srcW / 2;
	for (size_t row = 0; row < dstUVH; ++row)
	{
		const uint8_t* srcRow = srcUV + (row * srcUVH / dstUVH) * srcW;
		uint8_t* dstRow = dstUV + row * dstPitch;
		for (size_t pair = 0; pair < dstPairs; ++pair)
		{
			const size_t sourcePair = pair * (srcPairs - 1) / (dstPairs - 1);
			dstRow[pair * 2 + 0] = srcRow[sourcePair * 2 + 0]; // U
			dstRow[pair * 2 + 1] = srcRow[sourcePair * 2 + 1]; // V
		}
	}
	return want;
}

bool TwinpadCameraDecoder::HasFrame() const
{
	std::scoped_lock lock(m_mutex);
	return !m_nv12.empty();
}
