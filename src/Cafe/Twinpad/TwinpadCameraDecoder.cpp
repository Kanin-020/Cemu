#include "Common/precompiled.h"
#include "Cafe/Twinpad/TwinpadCameraDecoder.h"

#include <rapidjson/document.h>

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
		return false;
	rapidjson::Document document;
	document.Parse(line.c_str(), line.size());
	if (document.HasParseError() || !document.IsObject())
		return false;
	if (!document.HasMember("type") || !document["type"].IsString()
		|| std::string_view(document["type"].GetString()) != "camera_frame")
		return false;
	if (!document.HasMember("w") || !document["w"].IsUint() || document["w"].GetUint() != kWidth)
		return false;
	if (!document.HasMember("h") || !document["h"].IsUint() || document["h"].GetUint() != kHeight)
		return false;
	if (!document.HasMember("data") || !document["data"].IsString())
		return false;

	std::vector<uint8_t> accessUnit;
	if (!DecodeBase64Url(document["data"].GetString(), document["data"].GetStringLength(), accessUnit))
		return false;

	av_packet_unref(m_impl->packet);
	if (av_new_packet(m_impl->packet, static_cast<int>(accessUnit.size())) < 0)
		return false;
	std::memcpy(m_impl->packet->data, accessUnit.data(), accessUnit.size());

	if (avcodec_send_packet(m_impl->context, m_impl->packet) < 0)
		return false;

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

bool TwinpadCameraDecoder::HasFrame() const
{
	std::scoped_lock lock(m_mutex);
	return !m_nv12.empty();
}
