#include "TwinpadClient.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <cerrno>
#include <cstdio>
#include <optional>
#include <string_view>
#include <memory>
#include <cstring>
#include <filesystem>
#include <cstdlib>
#include <deque>
#include <rapidjson/document.h>
#include <wels/codec_api.h>
extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavcodec/bsf.h>
#include <libavcodec/defs.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/opt.h>
}
#include "Cafe/CafeSystem.h"
#include "Cafe/Twinpad/TwinpadCameraDecoder.h"
#include "Cafe/HW/Latte/Renderer/Renderer.h"

#if BOOST_OS_WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketLength = int;
using SocketHandle = SOCKET;
static void CloseSocket(SocketHandle socket) { if (socket >= 0) closesocket(socket); }
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <unistd.h>
#include <fcntl.h>
using SocketLength = socklen_t;
using SocketHandle = int;
static void CloseSocket(SocketHandle socket) { if (socket >= 0) close(socket); }
#endif

// CR-043: conversión RGB8→YUV acelerada con SSSE3 (runtime dispatch; sin
// soporte se usa el fallback escalar, que conserva el comportamiento exacto
// previo). No cambia el wire: solo el coste por frame del pipeline.
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#define TWINPAD_YUV_X86 1
#include <immintrin.h>
#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>
#endif
#endif

#ifdef TWINPAD_E2E_TEST_HOOK
#include <atomic>
#include <string>
#endif

static bool WaitForSocketWritable(SocketHandle socket)
{
	for (;;)
	{
		fd_set writable;
		FD_ZERO(&writable);
		FD_SET(socket, &writable);
		timeval timeout{2, 0};
#if BOOST_OS_WINDOWS
		const int ready = select(0, nullptr, &writable, nullptr, &timeout);
		if (ready == SOCKET_ERROR && WSAGetLastError() == WSAEINTR)
#else
		const int ready = select(socket + 1, nullptr, &writable, nullptr, &timeout);
		if (ready < 0 && errno == EINTR)
#endif
			continue;
		return ready > 0;
	}
}

namespace
{
constexpr uint16_t kGameIpcPort = 8421;
constexpr uint16_t kAudioIpcPort = 8422;
constexpr uint16_t kQrIpcPort = 8423;
#ifdef TWINPAD_E2E_TEST_HOOK
constexpr uint16_t kE2eCaptureIpcPort = 8424;
constexpr size_t kE2eRequestLimit = 128;
#endif
constexpr size_t kMaxQrResponseBytes = 16 * 1024;
constexpr uint64_t kRetryIntervalMs = 2000;
constexpr uint64_t kIdrIntervalMs = 2000;
constexpr uint64_t kVideoMetricLogIntervalMs = 5'000;
constexpr uint32_t kOpenH264SliceCount = 4;
constexpr uint32_t kGyroLogReportInterval = 10;
constexpr char kBase64Alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
constexpr uint8_t kH264SpsNalType = 7;
constexpr uint8_t kH264PpsNalType = 8;
constexpr uint8_t kH264IdrNalType = 5;
constexpr uint8_t kAnnexBStartCode[] = {0, 0, 0, 1};

size_t FindAnnexBStartCode(const uint8_t* bytes, size_t size, size_t from, size_t& prefixSize)
{
	for (size_t index = from; index + 3 <= size; ++index)
	{
		if (bytes[index] != 0 || bytes[index + 1] != 0)
			continue;
		if (bytes[index + 2] == 1)
		{
			prefixSize = 3;
			return index;
		}
		if (index + 4 <= size && bytes[index + 2] == 0 && bytes[index + 3] == 1)
		{
			prefixSize = 4;
			return index;
		}
	}
	return size;
}

bool ContainsH264Nal(const uint8_t* bytes, size_t size, uint8_t nalType)
{
	size_t prefixSize = 0;
	size_t start = FindAnnexBStartCode(bytes, size, 0, prefixSize);
	while (start < size)
	{
		const size_t nal = start + prefixSize;
		if (nal < size && (bytes[nal] & 0x1f) == nalType)
			return true;
		size_t nextPrefixSize = 0;
		start = FindAnnexBStartCode(bytes, size, nal + 1, nextPrefixSize);
		prefixSize = nextPrefixSize;
	}
	return false;
}

// CR-040 verificación: payload del primer NAL del tipo pedido (sin start code),
// vacío si no existe. Las cabeceras de SPS necesarios caben con holgura.
const std::vector<uint8_t> FirstH264Nal(const std::vector<uint8_t>& annexB, uint8_t nalType)
{
	size_t prefixSize = 0;
	size_t nextPrefixSize = 0;
	size_t start = FindAnnexBStartCode(annexB.data(), annexB.size(), 0, prefixSize);
	while (start < annexB.size())
	{
		const size_t nal = start + prefixSize;
		if (nal < annexB.size() && (annexB[nal] & 0x1f) == nalType)
		{
			const size_t next = FindAnnexBStartCode(annexB.data(), annexB.size(), nal + 1, nextPrefixSize);
			return std::vector<uint8_t>(annexB.begin() + nal, annexB.begin() + std::min(next, annexB.size()));
		}
		start = FindAnnexBStartCode(annexB.data(), annexB.size(), nal + 1, nextPrefixSize);
		prefixSize = nextPrefixSize;
	}
	return {};
}

bool ExtractH264ParameterSets(const uint8_t* extradata, size_t size, std::vector<uint8_t>& output)
{
	output.clear();
	if (!extradata || size == 0)
		return false;
	if (extradata[0] == 1 && size >= 7)
	{
		size_t offset = 5;
		const uint8_t spsCount = extradata[offset++] & 0x1f;
		for (uint8_t index = 0; index < spsCount; ++index)
		{
			if (offset + 2 > size)
				return false;
			const size_t nalSize = (static_cast<size_t>(extradata[offset]) << 8) | extradata[offset + 1];
			offset += 2;
			if (nalSize == 0 || nalSize > size - offset)
				return false;
			if ((extradata[offset] & 0x1f) == kH264SpsNalType)
			{
				output.insert(output.end(), std::begin(kAnnexBStartCode), std::end(kAnnexBStartCode));
				output.insert(output.end(), extradata + offset, extradata + offset + nalSize);
			}
			offset += nalSize;
		}
		if (offset >= size)
			return false;
		const uint8_t ppsCount = extradata[offset++];
		for (uint8_t index = 0; index < ppsCount; ++index)
		{
			if (offset + 2 > size)
				return false;
			const size_t nalSize = (static_cast<size_t>(extradata[offset]) << 8) | extradata[offset + 1];
			offset += 2;
			if (nalSize == 0 || nalSize > size - offset)
				return false;
			if ((extradata[offset] & 0x1f) == kH264PpsNalType)
			{
				output.insert(output.end(), std::begin(kAnnexBStartCode), std::end(kAnnexBStartCode));
				output.insert(output.end(), extradata + offset, extradata + offset + nalSize);
			}
			offset += nalSize;
		}
	}
	else
	{
		size_t prefixSize = 0;
		size_t start = FindAnnexBStartCode(extradata, size, 0, prefixSize);
		while (start < size)
		{
			const size_t nal = start + prefixSize;
			size_t nextPrefixSize = 0;
			const size_t next = FindAnnexBStartCode(extradata, size, nal + 1, nextPrefixSize);
			const size_t nalEnd = next < size ? next : size;
			if (nal < nalEnd && ((extradata[nal] & 0x1f) == kH264SpsNalType
				|| (extradata[nal] & 0x1f) == kH264PpsNalType))
			{
				output.insert(output.end(), std::begin(kAnnexBStartCode), std::end(kAnnexBStartCode));
				output.insert(output.end(), extradata + nal, extradata + nalEnd);
			}
			start = next;
			prefixSize = nextPrefixSize;
		}
	}
	return ContainsH264Nal(output.data(), output.size(), kH264SpsNalType)
		&& ContainsH264Nal(output.data(), output.size(), kH264PpsNalType);
}

bool HasH264ContractProfile(const std::vector<uint8_t>& annexB);

struct TwinpadFfmpegEncoder
{
	AVCodecContext* context{};
	AVFrame* frame{};
	AVFrame* softwareFrame{};
	AVBufferRef* hwDevice{};
	AVBufferRef* hwFrames{};
	bool hardwareFrames{};
	bool nv12{};
	bool annexBFilterEnabled{};
	AVPacket* packet{};
	AVBSFContext* annexBFilter{};
	bool bufferedOutput{}; // el último Encode() devolvió EAGAIN: el frame sigue en pipeline (saldrá más tarde)
	std::vector<uint8_t> annexBParameterSets;
	std::string name;

	~TwinpadFfmpegEncoder()
	{
		av_bsf_free(&annexBFilter);
		av_packet_free(&packet);
		av_frame_free(&softwareFrame);
		av_frame_free(&frame);
		av_buffer_unref(&hwFrames);
		av_buffer_unref(&hwDevice);
		avcodec_free_context(&context);
	}

	// CR-043: el frame de entrada (RGB8) ya se convirtió a NV12 consolidado
	// (Y + UV intercalado); data[] se apunta al buffer en lugar de copiar por
	// filas ni interleavear croma por pixel. Todos los candidatos hardware
	// (NVENC/QSV/AMF/VAAPI/VideoToolbox) consumen NV12.
	bool Encode(const uint8_t* nv12Y, const uint8_t* nv12Chroma,
		int width, int height, int64_t frameIndex, bool forceIdr,
		std::vector<uint8_t>& output)
	{
		if (!softwareFrame || !context || !packet || (hardwareFrames && !frame))
			return false;
		softwareFrame->data[0] = const_cast<uint8_t*>(nv12Y);
		softwareFrame->data[1] = const_cast<uint8_t*>(nv12Chroma);
		softwareFrame->data[2] = nullptr;
		softwareFrame->linesize[0] = width;
		softwareFrame->linesize[1] = width;
		softwareFrame->linesize[2] = 0;
		softwareFrame->pts = frameIndex;
		softwareFrame->pict_type = forceIdr ? AV_PICTURE_TYPE_I : AV_PICTURE_TYPE_NONE;
		if (forceIdr)
			softwareFrame->flags |= AV_FRAME_FLAG_KEY;
		else
			softwareFrame->flags &= ~AV_FRAME_FLAG_KEY;
		softwareFrame->color_range = AVCOL_RANGE_MPEG;

		AVFrame* frameToEncode = softwareFrame;
		if (hardwareFrames)
		{
			av_frame_unref(frame);
			if (!context->hw_frames_ctx || av_hwframe_get_buffer(context->hw_frames_ctx, frame, 0) < 0
				|| av_hwframe_transfer_data(frame, softwareFrame, 0) < 0)
			{
				av_frame_unref(frame);
				return false;
			}
			frame->pts = frameIndex;
			frame->pict_type = softwareFrame->pict_type;
			if (forceIdr)
				frame->flags |= AV_FRAME_FLAG_KEY;
			else
				frame->flags &= ~AV_FRAME_FLAG_KEY;
			frameToEncode = frame;
		}
		else if (softwareFrame->format != context->pix_fmt)
		{
			return false;
		}		av_packet_unref(packet);
		if (avcodec_send_frame(context, frameToEncode) < 0)
		{
			if (hardwareFrames)
				av_frame_unref(frame);
			return false;
		}
		int receiveResult = avcodec_receive_packet(context, packet);
		if (receiveResult == AVERROR(EAGAIN))
		{
			// El encoder hardware almacena el frame en su pipeline interno
			// (p. ej. VAAPI con asincronía): NO falla, emitirá la AU más tarde.
			bufferedOutput = true;
			if (hardwareFrames)
				av_frame_unref(frame);
			return false;
		}
		bufferedOutput = false;
		if (receiveResult < 0)
		{
			if (hardwareFrames)
				av_frame_unref(frame);
			return false;
		}
		if (hardwareFrames)
			av_frame_unref(frame);
		if (annexBFilterEnabled)
		{
			if (av_bsf_send_packet(annexBFilter, packet) < 0)
			{
				av_packet_unref(packet);
				return false;
			}
			av_packet_unref(packet);
			if (av_bsf_receive_packet(annexBFilter, packet) < 0)
			{
				av_packet_unref(packet);
				return false;
			}
		}
		if (packet->size <= 0 || packet->size > 4 * 1024 * 1024)
		{
			av_packet_unref(packet);
			return false;
		}
		if (ContainsH264Nal(packet->data, static_cast<size_t>(packet->size), kH264IdrNalType)
			&& (!ContainsH264Nal(packet->data, static_cast<size_t>(packet->size), kH264SpsNalType)
				|| !ContainsH264Nal(packet->data, static_cast<size_t>(packet->size), kH264PpsNalType))
			&& !annexBParameterSets.empty())
		{
			std::vector<uint8_t> parameterSetsAndFrame = annexBParameterSets;
			parameterSetsAndFrame.insert(parameterSetsAndFrame.end(), packet->data, packet->data + packet->size);
			av_packet_unref(packet);
			if (av_new_packet(packet, static_cast<int>(parameterSetsAndFrame.size())) < 0)
				return false;
			std::memcpy(packet->data, parameterSetsAndFrame.data(), parameterSetsAndFrame.size());
		}
		output.assign(packet->data, packet->data + packet->size);
		av_packet_unref(packet);
		if (forceIdr && (!ContainsH264Nal(output.data(), output.size(), kH264IdrNalType)
			|| !ContainsH264Nal(output.data(), output.size(), kH264SpsNalType)
			|| !ContainsH264Nal(output.data(), output.size(), kH264PpsNalType)
			|| !HasH264ContractProfile(output)))
			return false;
		return !output.empty();
	}
};

// CR-039: en Linux, la variante GNU/libre del driver VA-API de AMD no publica
// el entrypoint de encode H.264; el driver completo (RPM Fusion Free,
// mesa-va-drivers-freeworld) solo se resuelve si libva recibe
// LIBVA_DRIVERS_PATH. Tras un primer fallo de creación del device, exporta
// el directorio freeworld (instalado o junto al ejecutable) y reintenta una vez.
static bool CreateVaapiHwDevice(AVBufferRef** device)
{
	if (av_hwdevice_ctx_create(device, AV_HWDEVICE_TYPE_VAAPI, nullptr, nullptr, 0) == 0)
		return true;
#if BOOST_OS_LINUX
	static bool sFreeworldPathExported = false;
	if (sFreeworldPathExported)
		return false;
	sFreeworldPathExported = true;
	std::string searchPath;
	std::array<char, 4096> exePath{};
	const ssize_t exeLength = readlink("/proc/self/exe", exePath.data(), exePath.size() - 1);
	if (exeLength > 0)
	{
		exePath[exeLength] = '\0';
		const std::filesystem::path exeDir = std::filesystem::path(exePath.data()).parent_path();
		searchPath = (exeDir / "va-freeworld").string();
	}
	searchPath += ":/usr/lib64/dri-freeworld";
	setenv("LIBVA_DRIVERS_PATH", searchPath.c_str(), 0); // respeta un valor externo ya definido
	cemuLog_log(LogType::Force, "[TwinpadVideo] VAAPI device creation failed; retrying with freeworld drivers ({})",
		searchPath);
	return av_hwdevice_ctx_create(device, AV_HWDEVICE_TYPE_VAAPI, nullptr, nullptr, 0) == 0;
#else
	return false;
#endif
}

std::unique_ptr<TwinpadFfmpegEncoder> CreateHardwareH264Encoder(uint16_t width, uint16_t height, uint8_t fps, uint32_t bitrateKbps)
{
#if BOOST_OS_WINDOWS
	static constexpr const char* candidates[] = {"h264_nvenc", "h264_qsv", "h264_amf"};
#elif BOOST_OS_MACOS
	static constexpr const char* candidates[] = {"h264_videotoolbox"};
#elif BOOST_OS_LINUX
	static constexpr const char* candidates[] = {"h264_nvenc", "h264_qsv", "h264_amf", "h264_vaapi"};
#else
	return nullptr;
#endif
	for (const char* candidateName : candidates)
	{
		const AVCodec* codec = avcodec_find_encoder_by_name(candidateName);
		if (!codec)
			continue;
		auto encoder = std::make_unique<TwinpadFfmpegEncoder>();
		encoder->name = candidateName;
		encoder->context = avcodec_alloc_context3(codec);
		if (!encoder->context)
			continue;
		auto* context = encoder->context;
		context->width = width;
		context->height = height;
		const bool qsv = encoder->name == "h264_qsv";
		const bool vaapi = encoder->name == "h264_vaapi";
		encoder->hardwareFrames = qsv || vaapi;
		encoder->nv12 = encoder->hardwareFrames || encoder->name == "h264_amf"
			|| encoder->name == "h264_videotoolbox";
		const AVPixelFormat hardwarePixelFormat = vaapi ? AV_PIX_FMT_VAAPI : AV_PIX_FMT_QSV;
		context->pix_fmt = encoder->hardwareFrames ? hardwarePixelFormat
			: (encoder->nv12 ? AV_PIX_FMT_NV12 : AV_PIX_FMT_YUV420P);
		if (encoder->hardwareFrames)
		{
			const AVHWDeviceType deviceType = vaapi ? AV_HWDEVICE_TYPE_VAAPI : AV_HWDEVICE_TYPE_QSV;
			bool deviceCreated = false;
			if (vaapi)
				deviceCreated = CreateVaapiHwDevice(&encoder->hwDevice);
			else
				deviceCreated = av_hwdevice_ctx_create(&encoder->hwDevice, deviceType, nullptr, nullptr, 0) == 0;
			if (!deviceCreated)
				continue;
			encoder->hwFrames = av_hwframe_ctx_alloc(encoder->hwDevice);
			if (!encoder->hwFrames)
				continue;
			auto* frames = reinterpret_cast<AVHWFramesContext*>(encoder->hwFrames->data);
			frames->format = hardwarePixelFormat;
			frames->sw_format = AV_PIX_FMT_NV12;
			frames->width = width;
			frames->height = height;
			frames->initial_pool_size = 8;
			if (av_hwframe_ctx_init(encoder->hwFrames) < 0)
			{
				cemuLog_log(LogType::Force, "[TwinpadVideo] hardware candidate {} discarded: hw frames context init failed ({}x{} NV12, pool=8)",
					encoder->name, width, height);
				continue;
			}
			context->hw_frames_ctx = av_buffer_ref(encoder->hwFrames);
			if (!context->hw_frames_ctx)
				continue;
		}
		context->time_base = AVRational{1, fps};
		context->framerate = AVRational{fps, 1};
		context->bit_rate = bitrateKbps * 1000;
		context->rc_max_rate = bitrateKbps * 1000;
		context->rc_buffer_size = bitrateKbps * 1000;
		context->color_range = AVCOL_RANGE_MPEG;
		context->gop_size = fps * 2;
		context->max_b_frames = 0;			// CR-041: VAAPI negocia el perfil disponible del driver (Baseline o Main en
			// radeonsi freeworld); los demás backends siguen pidiendo Constrained Baseline.
			context->profile = vaapi ? AV_PROFILE_UNKNOWN : AV_PROFILE_H264_CONSTRAINED_BASELINE;
			context->level = 42;
		context->thread_count = 1;
		context->flags |= AV_CODEC_FLAG_LOW_DELAY;
		if (!vaapi)
			context->flags |= AV_CODEC_FLAG_GLOBAL_HEADER;
		AVDictionary* options = nullptr;
		if (encoder->name == "h264_nvenc")
		{
			av_dict_set(&options, "preset", "p1", 0);
			av_dict_set(&options, "tune", "ll", 0);
			av_dict_set(&options, "forced-idr", "1", 0);
		}
		else if (encoder->name == "h264_videotoolbox")
		{
			av_dict_set(&options, "realtime", "1", 0);
			av_dict_set(&options, "allow_sw", "0", 0);
			av_dict_set(&options, "profile", "constrained_baseline", 0);
		}
		else if (encoder->name == "h264_qsv")
		{
			av_dict_set(&options, "preset", "veryfast", 0);
			av_dict_set(&options, "async_depth", "1", 0);
			av_dict_set(&options, "forced_idr", "1", 0);
			av_dict_set(&options, "profile", "constrained_baseline", 0);
		}			else if (encoder->name == "h264_vaapi")
			{
				av_dict_set(&options, "rc_mode", "CBR", 0);
				// CR-041: "constrained_baseline" rebota en el driver freeworld (no lo
				// expone: No usable encoding profile found). Sin la opción el driver
				// negocia High (100), fuera del contrato. Pedimos Main (77): si el
				// driver no lo soporta, el descarte queda fail-loud en el log.
				av_dict_set(&options, "profile", "main", 0);
				// Minimizar el pipeline interno del encoder para que la IA salga
				// (casi) en el mismo tick que se envía el frame (latencia baja).
				av_dict_set(&options, "async_depth", "1", 0);
			}
		else if (encoder->name == "h264_amf")
		{
			av_dict_set(&options, "usage", "ultralowlatency", 0);
			av_dict_set(&options, "bf", "0", 0);
			av_dict_set(&options, "profile", "constrained_baseline", 0);
		}			const int openResult = avcodec_open2(context, codec, &options);
			const bool optionsAccepted = av_dict_count(options) == 0;
			av_dict_free(&options);
			// CR-041: el candidato negociado debe caer en el contrato del stream
			// (Constrained Baseline 578, Baseline 66 o Main 77); solo los backends
			// no-VAAPI fuerzan Constrained Baseline desde la configuración.
			const bool contractProfile = context->profile == AV_PROFILE_H264_CONSTRAINED_BASELINE
				|| context->profile == AV_PROFILE_H264_BASELINE
				|| context->profile == AV_PROFILE_H264_MAIN;
			if (openResult < 0 || !optionsAccepted
				|| !contractProfile
				|| context->level <= 0 || context->level > 42 || context->max_b_frames != 0)
			{
				// CR-040 verificación: motivo exacto del descarte del candidato (CR-041 diagnóstico de creación).
				cemuLog_log(LogType::Force, "[TwinpadVideo] hardware candidate {} discarded at open: result={} options_left={} negotiated profile={} level={} b_frames={} (contract: profile 578/66/77, level ≤42, b=0)",
					candidateName, openResult, optionsAccepted ? 0 : 1, context->profile, context->level, context->max_b_frames);
				continue;
			}
		encoder->frame = av_frame_alloc();
		encoder->softwareFrame = av_frame_alloc();
		encoder->packet = av_packet_alloc();
		if (!encoder->frame || !encoder->softwareFrame || !encoder->packet)
			continue;
		const size_t extraDataSize = static_cast<size_t>(std::max(context->extradata_size, 0));
		const bool avcConfiguration = extraDataSize > 0 && context->extradata[0] == 1;
		const bool hasParameterSets = avcConfiguration
			? ExtractH264ParameterSets(context->extradata, extraDataSize, encoder->annexBParameterSets)
			: extraDataSize == 0;			if (avcConfiguration && (!hasParameterSets
				|| !HasH264ContractProfile(encoder->annexBParameterSets)))
			{
				const auto& parameterSets = encoder->annexBParameterSets;
				if (hasParameterSets && !parameterSets.empty())
				{
					const auto sps = FirstH264Nal(parameterSets, kH264SpsNalType);
					if (sps.size() >= 4)
						cemuLog_log(LogType::Force, "[TwinpadVideo] hardware candidate {} discarded: extradata SPS reads profile_idc={} constraint_flags=0x{:02x} level_idc={} (contract: profile 66/77, level ≤42)",
							encoder->name, static_cast<unsigned>(sps[1]), static_cast<unsigned>(sps[2]), static_cast<unsigned>(sps[3]));
				}
				else
					cemuLog_log(LogType::Force, "[TwinpadVideo] hardware candidate {} discarded: extradata has no parseable SPS (avc={}, size={})",
						encoder->name, avcConfiguration ? 1 : 0, extraDataSize);
				continue;
			}
		if (avcConfiguration)
		{
			const AVBitStreamFilter* filter = av_bsf_get_by_name("h264_mp4toannexb");
			if (!filter || !hasParameterSets || av_bsf_alloc(filter, &encoder->annexBFilter) < 0
				|| avcodec_parameters_from_context(encoder->annexBFilter->par_in, context) < 0)
				continue;
			av_freep(&encoder->annexBFilter->par_in->extradata);
			encoder->annexBFilter->par_in->extradata = static_cast<uint8_t*>(
				av_mallocz(context->extradata_size + AV_INPUT_BUFFER_PADDING_SIZE));
			if (!encoder->annexBFilter->par_in->extradata)
				continue;
			std::memcpy(encoder->annexBFilter->par_in->extradata, context->extradata, context->extradata_size);
			encoder->annexBFilter->par_in->extradata_size = context->extradata_size;
			encoder->annexBFilter->time_base_in = context->time_base;
			if (av_bsf_init(encoder->annexBFilter) < 0)
				continue;
			encoder->annexBFilterEnabled = true;
		}
		encoder->softwareFrame->format = encoder->hardwareFrames
			? AV_PIX_FMT_NV12
			: (context->pix_fmt == AV_PIX_FMT_NV12 ? AV_PIX_FMT_NV12 : AV_PIX_FMT_YUV420P);
		encoder->softwareFrame->width = width;
		encoder->softwareFrame->height = height;
		// CR-043: sin av_frame_get_buffer — Encode apunta data[] al buffer NV12
		// del caller (zero-copy); la reserva propia quedó muerta.
		return encoder;
	}
	return nullptr;
}

ISVCEncoder* CreateOpenH264Encoder(uint16_t width, uint16_t height, uint8_t fps, uint32_t bitrateKbps)
{
	ISVCEncoder* encoder = nullptr;
	if (WelsCreateSVCEncoder(&encoder) != 0 || !encoder)
		return nullptr;
	SEncParamExt params{};
	if (encoder->GetDefaultParams(&params) != 0)
	{
		WelsDestroySVCEncoder(encoder);
		return nullptr;
	}
	params.iUsageType = SCREEN_CONTENT_REAL_TIME;
	params.iComplexityMode = LOW_COMPLEXITY;
	params.iPicWidth = width;
	params.iPicHeight = height;
	params.iTargetBitrate = bitrateKbps * 1000;
	params.iRCMode = RC_BITRATE_MODE;
	params.fMaxFrameRate = static_cast<float>(fps);
	params.iSpatialLayerNum = 1;
	params.iTemporalLayerNum = 1;
	params.uiIntraPeriod = fps * 2;
	params.bEnableFrameSkip = false;
	params.bEnableSceneChangeDetect = false;
	params.eSpsPpsIdStrategy = CONSTANT_ID;
	params.iEntropyCodingModeFlag = 0;
	params.sSpatialLayers[0].iVideoWidth = width;
	params.sSpatialLayers[0].iVideoHeight = height;
	params.sSpatialLayers[0].fFrameRate = static_cast<float>(fps);
	params.sSpatialLayers[0].iSpatialBitrate = bitrateKbps * 1000;
	params.sSpatialLayers[0].iMaxSpatialBitrate = bitrateKbps * 1000;
	params.sSpatialLayers[0].uiProfileIdc = PRO_BASELINE;
	params.sSpatialLayers[0].uiLevelIdc = LEVEL_4_2;
	params.sSpatialLayers[0].sSliceArgument.uiSliceMode = SM_FIXEDSLCNUM_SLICE;
	params.sSpatialLayers[0].sSliceArgument.uiSliceNum = kOpenH264SliceCount;
	params.iMultipleThreadIdc = static_cast<unsigned short>(kOpenH264SliceCount);
	if (encoder->InitializeExt(&params) != 0)
	{
		WelsDestroySVCEncoder(encoder);
		return nullptr;
	}
	int colorFormat = videoFormatI420;
	if (encoder->SetOption(ENCODER_OPTION_DATAFORMAT, &colorFormat) != 0)
	{
		encoder->Uninitialize();
		WelsDestroySVCEncoder(encoder);
		return nullptr;
	}
	return encoder;
}

bool ContainsIdrNal(const std::vector<uint8_t>& annexB)
{
	return ContainsH264Nal(annexB.data(), annexB.size(), kH264IdrNalType);
}

// ===== CR-043: conversión RGB8→YUV (I420/NV12) =====
// MISMA aritmética de punto fijo y redondeos que la ruta escalar previa:
// y_i = ((66R+129G+25B+128) >> 8) + 16
// u_i = ((-38R-74G+112B+128) >> 8) + 128, croma = promedio de los 4 u_i del
// bloque 2x2 (floor /4), pinchado a 0..255 (el rango ya está en 16..239).
// Ambas rutas (escalar y SIMD) producen resultados idénticos bit a bit.

#if defined(TWINPAD_YUV_X86) && (defined(__GNUC__) || defined(__clang__))
#define TWINPAD_YUV_TARGET __attribute__((target("ssse3")))
#else
#define TWINPAD_YUV_TARGET
#endif

bool HasSsse3Support()
{
#ifdef TWINPAD_YUV_X86
#if defined(__GNUC__) || defined(__clang__)
	return __builtin_cpu_supports("ssse3") != 0;
#elif defined(_MSC_VER)
	int cpuInfo[4] = {};
	__cpuid(cpuInfo, 1);
	return (cpuInfo[2] & (1 << 9)) != 0;
#else
	return false;
#endif
#else
	return false;
#endif
}

// Promedio 2x2 de croma para el rango de columnas [columnStart, columnEnd).
// Con nv12=true produce UV intercalado (ancha de fila: width bytes); con
// nv12=false escribe planos U y V separados (una fila: width/2 bytes).
void ConvertChromaScalarRange(const uint8_t* rgb, int width, int height, bool nv12, uint8_t* dst, int columnStart, int columnEnd)
{
	const size_t ySize = static_cast<size_t>(width) * height;
	for (int row = 0; row < height; row += 2)
	{
		const size_t chromaRow = static_cast<size_t>(row / 2) * (nv12 ? width : width / 2);
		for (int column = columnStart; column < columnEnd; column += 2)
		{
			int sumU = 0;
			int sumV = 0;
			for (int dy = 0; dy < 2; ++dy)
			{
				for (int dx = 0; dx < 2; ++dx)
				{
					const size_t source = (static_cast<size_t>(row + dy) * width + column + dx) * 3;
					const int red = rgb[source];
					const int green = rgb[source + 1];
					const int blue = rgb[source + 2];
					sumU += ((-38 * red - 74 * green + 112 * blue + 128) >> 8) + 128;
					sumV += ((112 * red - 94 * green - 18 * blue + 128) >> 8) + 128;
				}
			}
			const uint8_t uValue = static_cast<uint8_t>(std::clamp(sumU / 4, 0, 255));
			const uint8_t vValue = static_cast<uint8_t>(std::clamp(sumV / 4, 0, 255));
			if (nv12)
			{
				dst[ySize + chromaRow + column] = uValue;
				dst[ySize + chromaRow + column + 1] = vValue;
			}
			else
			{
				const size_t chroma = chromaRow + column / 2;
				dst[ySize + chroma] = uValue;
				dst[ySize + ySize / 4 + chroma] = vValue;
			}
		}
	}
}

void ConvertRgbToI420Scalar(const uint8_t* rgb, int width, int height, uint8_t* dst)
{
	for (int row = 0; row < height; ++row)
	{
		for (int column = 0; column < width; ++column)
		{
			const size_t source = (static_cast<size_t>(row) * width + column) * 3;
			const int red = rgb[source];
			const int green = rgb[source + 1];
			const int blue = rgb[source + 2];
			dst[static_cast<size_t>(row) * width + column] = static_cast<uint8_t>(
				((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
		}
	}
	ConvertChromaScalarRange(rgb, width, height, false, dst, 0, width);
}

void ConvertRgbToNv12Scalar(const uint8_t* rgb, int width, int height, uint8_t* dst)
{
	for (int row = 0; row < height; ++row)
	{
		for (int column = 0; column < width; ++column)
		{
			const size_t source = (static_cast<size_t>(row) * width + column) * 3;
			const int red = rgb[source];
			const int green = rgb[source + 1];
			const int blue = rgb[source + 2];
			dst[static_cast<size_t>(row) * width + column] = static_cast<uint8_t>(
				((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
		}
	}
	ConvertChromaScalarRange(rgb, width, height, true, dst, 0, width);
}

#ifdef TWINPAD_YUV_X86
// CR-043: conversión SSSE3. pshufb arma por dword los pares (r,g) (byte B
// aparte) y _mm_madd_epi16 aplica los pares de coeficientes BT.601: un dword
// por píxel en luma y croma. La carga de 16 B sale de la fila cuando
// 3x+15 < 3*width; los guards "x+6 ≤ width" (luma) y "column+6 ≤ width"
// (croma) acotan la ventana y las columnas de cola pasan por la ruta escalar.
TWINPAD_YUV_TARGET
static void ConvertRgbToYuvSsse3(const uint8_t* rgb, int width, int height, bool nv12, uint8_t* dst)
{
	const size_t ySize = static_cast<size_t>(width) * height;
	const __m128i zero = _mm_setzero_si128();
	// pshufb reordena BYTE a BYTE con copy directo del índice; el byte fuente
	// 0x80 (índice inválido) escribe 0. Cada dword del vector destino es
	// [r_i, 0, g_i, 0] (o b_i solo); interpretado como dos WORD little-endian:
	// w0 = r_i, w1 = g_i; _mm_madd_epi16 con par (66,129) evalúa w0*66 + w1*129
	// (BT.601 completo) en el dword i = 66*r_i + 129*g_i. Coef B (25) en otro
	// madd, con set1 (par constante → él solo el byte B).
	const __m128i rg4Mask = _mm_setr_epi8(0, static_cast<char>(0x80), 1, static_cast<char>(0x80),
		3, static_cast<char>(0x80), 4, static_cast<char>(0x80),
		6, static_cast<char>(0x80), 7, static_cast<char>(0x80),
		9, static_cast<char>(0x80), 10, static_cast<char>(0x80));
	// dword del canal B: [B_i, 0, 0, 0] (¡no en la posición 2! si B_{i+1}
	// cayera en ese dword, madd con set1 sumaría dos píxeles).
	const __m128i b4Mask = _mm_setr_epi8(2, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		5, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		8, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		11, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80));
	const __m128i rg2Mask = _mm_setr_epi8(0, static_cast<char>(0x80), 1, static_cast<char>(0x80),
		3, static_cast<char>(0x80), 4, static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80));
	const __m128i b2Mask = _mm_setr_epi8(2, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		5, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80));
	const __m128i yRGPair = _mm_setr_epi16(66, 129, 66, 129, 66, 129, 66, 129);
	const __m128i uRGPair = _mm_setr_epi16(-38, -74, -38, -74, -38, -74, -38, -74);
	const __m128i vRGPair = _mm_setr_epi16(112, -94, 112, -94, 112, -94, 112, -94);
	const __m128i yBCoeff = _mm_set1_epi16(25);
	const __m128i uBCoeff = _mm_set1_epi16(112);
	const __m128i vBCoeff = _mm_set1_epi16(-18);
	const __m128i roundBias = _mm_set1_epi32(128);
	const __m128i lumaLift = _mm_set1_epi32(16);
	const __m128i chromaLift = _mm_set1_epi32(128);
	// byte bajo del dword i (0x80 = escribe 0): Y queda en [16,235], el byte
	// bajo ES el valor; sin saturación (packss_epi32 sería SSE4.1).
	const __m128i lowBytesMask = _mm_setr_epi8(0, 4, 8, 12, static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80));

	for (int row = 0; row < height; ++row)
	{
		const uint8_t* line = rgb + static_cast<size_t>(row) * width * 3;
		uint8_t* dstY = dst + static_cast<size_t>(row) * width;
		int x = 0;
		// La ventana de 16 B llega hasta 3x+15; con x+6 ≤ width queda dentro de la fila.
		for (; x + 6 <= width; x += 4, dstY += 4)
		{
			const __m128i raw = _mm_loadu_si128(reinterpret_cast<const __m128i*>(line + x * 3));
			const __m128i rg = _mm_shuffle_epi8(raw, rg4Mask);
			const __m128i b = _mm_shuffle_epi8(raw, b4Mask);
			__m128i y32 = _mm_add_epi32(_mm_madd_epi16(rg, yRGPair), _mm_madd_epi16(b, yBCoeff));
			y32 = _mm_add_epi32(y32, roundBias);
			y32 = _mm_srli_epi32(y32, 8); // el pre-offset garantiza no negativos
			y32 = _mm_add_epi32(y32, lumaLift);
			// Sin saturación: Y ∈ [16,235] cabe en un byte pese a todo.
			const __m128i yBytes = _mm_shuffle_epi8(y32, lowBytesMask);
			std::memcpy(dstY, &yBytes, 4);
		}
		for (; x < width; ++x, ++dstY)
		{
			const int red = line[x * 3];
			const int green = line[x * 3 + 1];
			const int blue = line[x * 3 + 2];
			*dstY = static_cast<uint8_t>(((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
		}
	}

	// Croma SIMD por bloque 2x2; las columnas colas (x ≥ width-4) pasan por el
	// helper escalar al final (mismo redondeo).
	if (width >= 6)
	{
		for (int row = 0; row < height; row += 2)
		{
			const uint8_t* rowA = rgb + static_cast<size_t>(row) * width * 3;
			const uint8_t* rowB = rowA + static_cast<size_t>(width) * 3;
			const size_t chromaRow = static_cast<size_t>(row / 2) * (nv12 ? width : width / 2);
			uint8_t* const dstU = dst + ySize + chromaRow;
			uint8_t* const dstV = nv12 ? dstU + 1 : dst + ySize + ySize / 4 + chromaRow;
			for (int column = 0; column < width - 5; column += 2)
			{
				const __m128i rawA = _mm_loadu_si128(reinterpret_cast<const __m128i*>(rowA + column * 3));
				const __m128i rawB = _mm_loadu_si128(reinterpret_cast<const __m128i*>(rowB + column * 3));
				const __m128i rgA = _mm_shuffle_epi8(rawA, rg2Mask);
				const __m128i bA = _mm_shuffle_epi8(rawA, b2Mask);
				const __m128i rgB = _mm_shuffle_epi8(rawB, rg2Mask);
				const __m128i bB = _mm_shuffle_epi8(rawB, b2Mask);
				// u_i/v_i por píxel (BT.601) con el MISMO redondeo por píxel que la
				// ruta escalar: (grpc+128)>>8+128 por FILA (lane0 = (x,y), lane1 =
				// (x+1,y); después se suma la fila B en las mismas lanes).
				__m128i u32 = _mm_add_epi32(_mm_madd_epi16(rgA, uRGPair), _mm_madd_epi16(bA, uBCoeff));
				u32 = _mm_add_epi32(u32, roundBias);
				u32 = _mm_srai_epi32(u32, 8); // aritmético: el pre-offset puede ser negativo
				u32 = _mm_add_epi32(u32, chromaLift);
				u32 = _mm_add_epi32(u32, _mm_srai_epi32(_mm_add_epi32(
					_mm_add_epi32(_mm_madd_epi16(rgB, uRGPair), _mm_madd_epi16(bB, uBCoeff)), roundBias), 8));
				u32 = _mm_add_epi32(u32, chromaLift);
				__m128i v32 = _mm_add_epi32(_mm_madd_epi16(rgA, vRGPair), _mm_madd_epi16(bA, vBCoeff));
				v32 = _mm_add_epi32(v32, roundBias);
				v32 = _mm_srai_epi32(v32, 8);
				v32 = _mm_add_epi32(v32, chromaLift);
				v32 = _mm_add_epi32(v32, _mm_srai_epi32(_mm_add_epi32(
					_mm_add_epi32(_mm_madd_epi16(rgB, vRGPair), _mm_madd_epi16(bB, vBCoeff)), roundBias), 8));
				v32 = _mm_add_epi32(v32, chromaLift);
				// Suma horizontal exacta de los 4 valores en lane0: (lane0)+(lane1).
				const __m128i uSum = _mm_add_epi32(u32, _mm_shuffle_epi32(u32, _MM_SHUFFLE(0, 0, 0, 1)));
				const __m128i vSum = _mm_add_epi32(v32, _mm_shuffle_epi32(v32, _MM_SHUFFLE(0, 0, 0, 1)));
				// /4 lógico (cada u_i ∈ 16..239, la suma ∈ 64..956, no negativa);
				// el dword lane0 cabe en el byte (≤ 239) sin saturación extra.
				if (nv12)
				{
					dstU[column] = static_cast<uint8_t>(_mm_cvtsi128_si32(_mm_srli_epi32(uSum, 2)));
					dstU[column + 1] = static_cast<uint8_t>(_mm_cvtsi128_si32(_mm_srli_epi32(vSum, 2)));
				}
				else
				{
					dstU[column / 2] = static_cast<uint8_t>(_mm_cvtsi128_si32(_mm_srli_epi32(uSum, 2)));
					dstV[column / 2] = static_cast<uint8_t>(_mm_cvtsi128_si32(_mm_srli_epi32(vSum, 2)));
				}
			}
		}
	}
	ConvertChromaScalarRange(rgb, width, height, nv12, dst, width >= 6 ? width - 4 : 0, width);
}
#endif

void ConvertRgbToYuv(const uint8_t* rgb, int width, int height, bool nv12, std::vector<uint8_t>& dst)
{
	dst.resize(static_cast<size_t>(width) * height * 3 / 2);
#ifdef TWINPAD_YUV_X86
	static const bool hasSsse3 = HasSsse3Support();
	if (hasSsse3)
	{
		ConvertRgbToYuvSsse3(rgb, width, height, nv12, dst.data());
		return;
	}
#endif
	if (nv12)
		ConvertRgbToNv12Scalar(rgb, width, height, dst.data());
	else
		ConvertRgbToI420Scalar(rgb, width, height, dst.data());
}

// CR-041: contrato del stream relajado — Baseline (66, con o sin constraint_set0)
// o Main (77), nivel ≤4.2, YUV 4:2:0, sin B-frames. Constrained Baseline 4.2
// (66 + set0 + 42) sigue siendo válido como subconjunto. Motivo: el driver
// radeonsi freeworld no expone Constrained Baseline y el encoder VAAPI rebota
// definitivamente con el contrato estricto (log: negotiated profile=578, result=-38).
// level_idc usa byte directo para todos los niveles estándar; 0 no es un nivel válido.
bool HasH264ContractProfile(const std::vector<uint8_t>& annexB)
{
	size_t prefixSize = 0;
	size_t start = FindAnnexBStartCode(annexB.data(), annexB.size(), 0, prefixSize);
	while (start < annexB.size())
	{
		const size_t nal = start + prefixSize;
		if (nal + 3 < annexB.size() && (annexB[nal] & 0x1f) == kH264SpsNalType
			&& (annexB[nal + 1] == 66 || annexB[nal + 1] == 77)
			&& annexB[nal + 3] != 0 && annexB[nal + 3] <= 42)
			return true;
		size_t nextPrefixSize = 0;
		start = FindAnnexBStartCode(annexB.data(), annexB.size(), nal + 1, nextPrefixSize);
		prefixSize = nextPrefixSize;
	}
	return false;
}

uint8_t SrgbToRgb(uint8_t value)
{
	static const std::array<uint8_t, 256> linearLookup = [] {
		std::array<uint8_t, 256> lookup{};
		for (size_t index = 0; index < lookup.size(); ++index)
		{
			const float srgb = static_cast<float>(index) / 255.0f;
			const float linear = srgb <= 0.04045f ? srgb / 12.92f : std::pow((srgb + 0.055f) / 1.055f, 2.4f);
			lookup[index] = static_cast<uint8_t>(std::min(linear, 1.0f) * 255.0f);
		}
		return lookup;
	}();
	return linearLookup[value];
}

std::optional<TwinpadInputState> ParseInputReport(const rapidjson::Value& document)
{
	if (!document.IsObject() || !document.HasMember("type") || !document["type"].IsString()
		|| std::string_view(document["type"].GetString()) != "input_report"
		|| !document.HasMember("seq") || !document["seq"].IsUint()
		|| !document.HasMember("dpad") || !document["dpad"].IsUint()
		|| !document.HasMember("buttons") || !document["buttons"].IsUint()
		|| !document.HasMember("trigger_l2") || !document["trigger_l2"].IsUint()
		|| !document.HasMember("trigger_r2") || !document["trigger_r2"].IsUint()
		|| !document.HasMember("touch_x") || !document["touch_x"].IsUint()
		|| !document.HasMember("touch_y") || !document["touch_y"].IsUint()
		|| !document.HasMember("touch_pressed") || !document["touch_pressed"].IsBool()
		|| document.HasMember("extra_1") || document.HasMember("extra_2"))
		return std::nullopt;

	TwinpadInputState state{};
	auto getStick = [&document](const char* key, int8_t& output) {
		if (!document.HasMember(key) || !document[key].IsInt())
			return false;
		const int value = document[key].GetInt();
		if (value < -127 || value > 127)
			return false;
		output = static_cast<int8_t>(value);
		return true;
	};
	if (!getStick("stick_lx", state.stickLx) || !getStick("stick_ly", state.stickLy)
		|| !getStick("stick_rx", state.stickRx) || !getStick("stick_ry", state.stickRy))
		return std::nullopt;

	const uint32_t dpad = document["dpad"].GetUint();
	const uint32_t buttons = document["buttons"].GetUint();
	const uint32_t triggerL2 = document["trigger_l2"].GetUint();
	const uint32_t triggerR2 = document["trigger_r2"].GetUint();
	const uint32_t touchX = document["touch_x"].GetUint();
	const uint32_t touchY = document["touch_y"].GetUint();
	if (dpad > 8 || buttons > 0x0fff || triggerL2 > 255 || triggerR2 > 255
		|| touchX > 65535 || touchY > 65535)
		return std::nullopt;
	state.seq = document["seq"].GetUint();
	state.dpad = static_cast<uint8_t>(dpad);
	state.buttons = static_cast<uint16_t>(buttons);
	state.triggerL2 = static_cast<uint8_t>(triggerL2);
	state.triggerR2 = static_cast<uint8_t>(triggerR2);
	state.touchX = static_cast<uint16_t>(touchX);
	state.touchY = static_cast<uint16_t>(touchY);
	state.touchPressed = document["touch_pressed"].GetBool();

	if (document.HasMember("motion"))
	{
		for (auto member = document.MemberBegin(); member != document.MemberEnd(); ++member)
		{
			const std::string_view name(member->name.GetString(), member->name.GetStringLength());
			if (name == "motion")
				continue;
			if (name != "type" && name != "seq" && name != "stick_lx" && name != "stick_ly"
				&& name != "stick_rx" && name != "stick_ry" && name != "dpad" && name != "buttons"
				&& name != "trigger_l2" && name != "trigger_r2" && name != "touch_x" && name != "touch_y"
				&& name != "touch_pressed")
				return std::nullopt;
		}
		const auto& motion = document["motion"];
		if (!motion.IsObject() || motion.MemberCount() != 3
			|| !motion.HasMember("timestamp_us") || !motion["timestamp_us"].IsUint64()
			|| !motion.HasMember("accel") || !motion["accel"].IsArray() || motion["accel"].Size() != 3
			|| !motion.HasMember("gyro") || !motion["gyro"].IsArray() || motion["gyro"].Size() != 3)
			return std::nullopt;
		state.motionTimestampUs = motion["timestamp_us"].GetUint64();
		for (rapidjson::SizeType index = 0; index < 3; ++index)
		{
			if (!motion["accel"][index].IsNumber() || !motion["gyro"][index].IsNumber())
				return std::nullopt;
			const double accel = motion["accel"][index].GetDouble();
			const double gyro = motion["gyro"][index].GetDouble();
			if (std::abs(accel) > std::numeric_limits<float>::max()
				|| std::abs(gyro) > std::numeric_limits<float>::max())
				return std::nullopt;
			state.accel[index] = static_cast<float>(accel);
			state.gyro[index] = static_cast<float>(gyro);
			if (!std::isfinite(state.accel[index]) || !std::isfinite(state.gyro[index]))
				return std::nullopt;
		}
		if (!TwinpadMotionMapper::IsFinite(state))
			return std::nullopt;
		state.motionValid = true;
	}
	return state;
}
}

TwinpadClient& TwinpadClient::Instance()
{
	static TwinpadClient client;
	return client;
}

TwinpadClient::~TwinpadClient()
{
	Stop();
	ResetVideoEncoder();
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
	{
		delete static_cast<TwinpadFfmpegEncoder*>(m_hardwareVideoEncoder);
		m_hardwareVideoEncoder = nullptr;
	}
	if (m_videoEncoder)
	{
		auto* encoder = static_cast<ISVCEncoder*>(m_videoEncoder);
		encoder->Uninitialize();
		WelsDestroySVCEncoder(encoder);
		m_videoEncoder = nullptr;
	}		m_encoderWidth = 0;
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

bool TwinpadClient::Start(bool captureGamePad)
{
	std::scoped_lock lifecycleLock(m_lifecycleMutex);
	if (m_running.exchange(true))
		return true;
	m_captureGamePad.store(captureGamePad);
	m_lastFrameSentMs.store(0);
	m_audioThread = std::thread(&TwinpadClient::AudioLoop, this);
	m_frameThread = std::thread(&TwinpadClient::FrameLoop, this);
	m_inputThread = std::thread(&TwinpadClient::InputLoop, this);
	return true;
}

void TwinpadClient::Stop()
{
	{
		std::scoped_lock lifecycleLock(m_lifecycleMutex);
		m_titleActive.store(false);
		m_connectionRequested.store(false);
		m_connected.store(false);
		m_running.store(false);
	}
	m_audioQueueChanged.notify_all();
	m_frameQueueChanged.notify_all();
	{
		std::scoped_lock lock(m_primarySocketMutex);
		if (m_primarySocket >= 0)
		{
#if BOOST_OS_WINDOWS
			shutdown(static_cast<SocketHandle>(m_primarySocket), SD_BOTH);
#else
			shutdown(static_cast<SocketHandle>(m_primarySocket), SHUT_RDWR);
#endif
		}
	}
	{
		std::scoped_lock lock(m_audioSocketMutex);
		if (m_audioSocket >= 0)
		{
#if BOOST_OS_WINDOWS
			shutdown(static_cast<SocketHandle>(m_audioSocket), SD_BOTH);
#else
			shutdown(static_cast<SocketHandle>(m_audioSocket), SHUT_RDWR);
#endif
		}
	}
	if (m_audioThread.joinable())
		m_audioThread.join();
	if (m_frameThread.joinable())
		m_frameThread.join();
	if (m_inputThread.joinable())
		m_inputThread.join();
	{
		std::scoped_lock lock(m_frameQueueMutex);
		m_pendingFrame.clear();
		m_hasPendingFrame = false;
	}
	{
		std::scoped_lock lock(m_audioQueueMutex);
		m_audioQueue.clear();
	}
	{
		std::scoped_lock primaryLock(m_primarySocketMutex);
		if (m_primarySocket >= 0)
		{
			CloseSocket(static_cast<SocketHandle>(m_primarySocket));
			m_primarySocket = -1;
		}
	}
	std::scoped_lock audioLock(m_audioSocketMutex);
	if (m_audioSocket >= 0)
	{
		SendLine(m_audioSocket, "{\"type\":\"game_audio_stop\",\"game_id\":\"cemu\"}");
		CloseSocket(static_cast<SocketHandle>(m_audioSocket));
		m_audioSocket = -1;
	}
	m_connected.store(false);
	ResetVideoEncoder();
}

void TwinpadClient::SetTitleActive(bool active, bool connectAutomatically)
{
	{
		std::scoped_lock lifecycleLock(m_lifecycleMutex);
		m_titleActive.store(active);
		if (active && connectAutomatically)
		{
			m_connectionRequested.store(true);
			m_captureGamePad.store(true);
		}
		if (!active)
		{
			m_connectionRequested.store(false);
			m_captureGamePad.store(false);
		}
	}
	if (active && connectAutomatically && !m_running.load())
		Start(true);
	if (!active)
	{
		m_connected.store(false);
		{
			std::scoped_lock lock(m_primarySocketMutex);
			if (m_primarySocket >= 0)
			{
#if BOOST_OS_WINDOWS
				shutdown(static_cast<SocketHandle>(m_primarySocket), SD_BOTH);
#else
				shutdown(static_cast<SocketHandle>(m_primarySocket), SHUT_RDWR);
#endif
			}
		}
		{
			std::scoped_lock lock(m_audioSocketMutex);
			if (m_audioSocket >= 0)
			{
#if BOOST_OS_WINDOWS
				shutdown(static_cast<SocketHandle>(m_audioSocket), SD_BOTH);
#else
				shutdown(static_cast<SocketHandle>(m_audioSocket), SHUT_RDWR);
#endif
			}
		}
		ResetVideoEncoder();
	}
}

bool TwinpadClient::ConnectPrimary()
{
#if BOOST_OS_WINDOWS
	static const bool winsockReady = [] { WSADATA data{}; return WSAStartup(MAKEWORD(2, 2), &data) == 0; }();
	if (!winsockReady) return false;
#endif
	std::scoped_lock socketLock(m_primarySocketMutex);
	m_primarySocket = static_cast<std::intptr_t>(socket(AF_INET, SOCK_STREAM, 0));
	if (m_primarySocket < 0) return false;
	sockaddr_in endpoint{};
	endpoint.sin_family = AF_INET;
	endpoint.sin_port = htons(kGameIpcPort);
	inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
	if (connect(static_cast<SocketHandle>(m_primarySocket), reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0)
	{
		CloseSocket(static_cast<SocketHandle>(m_primarySocket));
		m_primarySocket = -1;
		return false;
	}
#if BOOST_OS_WINDOWS
	u_long nonBlocking = 1;
	ioctlsocket(static_cast<SocketHandle>(m_primarySocket), FIONBIO, &nonBlocking);
#else
	const int socketFlags = fcntl(static_cast<SocketHandle>(m_primarySocket), F_GETFL, 0);
	if (socketFlags >= 0)
		fcntl(static_cast<SocketHandle>(m_primarySocket), F_SETFL, socketFlags | O_NONBLOCK);
#endif
	const std::string announce = "{\"type\":\"game_announce\",\"game_id\":\"cemu\",\"game_name\":\"Cemu (experimental)\",\"capabilities\":[\"second_screen\",\"audio\"]}";

	if (!SendLine(m_primarySocket, announce))
	{
		CloseSocket(static_cast<SocketHandle>(m_primarySocket));
		m_primarySocket = -1;
		return false;
	}
	// CR-038: si la cámara ya estaba demandada, el canal recién conectado
	// informa de inmediato al Servidor. CR-035: idem micrófono.
	if (m_cameraDemandOn.load())
		SendLine(m_primarySocket, CameraRequestLine(true));
	if (m_micDemandOn.load())
		SendLine(m_primarySocket, MicRequestLine(true));
	return true;
}

bool TwinpadClient::ConnectAudio()
{
	std::scoped_lock socketLock(m_audioSocketMutex);
	m_audioSocket = static_cast<std::intptr_t>(socket(AF_INET, SOCK_STREAM, 0));
	if (m_audioSocket < 0) return false;
	sockaddr_in endpoint{};
	endpoint.sin_family = AF_INET;
	endpoint.sin_port = htons(kAudioIpcPort);
	inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
	if (connect(static_cast<SocketHandle>(m_audioSocket), reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0)
	{
		CloseSocket(static_cast<SocketHandle>(m_audioSocket));
		m_audioSocket = -1;
		return false;
	}
	if (!SendLine(m_audioSocket, "{\"type\":\"game_audio_start\",\"game_id\":\"cemu\",\"rate\":48000,\"channels\":2,\"sample_fmt\":\"s16le\"}"))
	{
		CloseSocket(static_cast<SocketHandle>(m_audioSocket));
		m_audioSocket = -1;
		return false;
	}
	return true;
}

bool TwinpadClient::SendLine(std::intptr_t socket, const std::string& line)
{	if (socket < 0)
		return false;
	const std::string payload = line + "\n";

	size_t sent = 0;
	while (sent < payload.size())
	{
#if BOOST_OS_WINDOWS
		const int count = ::send(static_cast<SocketHandle>(socket), payload.data() + sent, static_cast<int>(payload.size() - sent), 0);
		if (count == SOCKET_ERROR)
		{
			const int error = WSAGetLastError();
			if (error == WSAEINTR)
				continue;
			if (error != WSAEWOULDBLOCK || !WaitForSocketWritable(static_cast<SocketHandle>(socket)))
				return false;
			continue;
		}
#else
		const ssize_t count = ::send(static_cast<SocketHandle>(socket), payload.data() + sent, payload.size() - sent, MSG_NOSIGNAL);
		if (count < 0)
		{
			if (errno == EINTR)
				continue;
			if ((errno != EAGAIN && errno != EWOULDBLOCK)
				|| !WaitForSocketWritable(static_cast<SocketHandle>(socket)))
				return false;
			continue;
		}
#endif
		if (count == 0) return false;
		sent += static_cast<size_t>(count);
	}
	return true;
}

std::vector<uint8_t> TwinpadClient::ScaleRgb(
	const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height, uint16_t targetWidth, uint16_t targetHeight) // CR-030
{
	if (!width || !height || !targetWidth || !targetHeight || rgb.size() != static_cast<size_t>(width) * height * 3)
		return {};
	if (targetWidth >= width && targetHeight >= height)
		return rgb; // sin upscale (CR-030)
	std::vector<uint8_t> target(static_cast<size_t>(targetWidth) * targetHeight * 3);
	for (uint32_t targetY = 0; targetY < targetHeight; ++targetY)
	{
		const size_t sourceY = targetY * height / targetHeight;
		for (uint32_t targetX = 0; targetX < targetWidth; ++targetX)
		{
			const size_t sourceX = targetX * width / targetWidth;
			const size_t source = (sourceY * width + sourceX) * 3;
			const size_t destination = (targetY * targetWidth + targetX) * 3;
			target[destination] = rgb[source];
			target[destination + 1] = rgb[source + 1];
			target[destination + 2] = rgb[source + 2];
		}
	}
	return target;
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
	uint16_t cadenceFps = kEncoderNominalFps;
	if (m_encodeStampsUs.size() >= 2)
	{
		const uint64_t spanUs = nowUs - m_encodeStampsUs.front();
		if (spanUs > 0)
		{
			cadenceFps = static_cast<uint16_t>(std::clamp<uint64_t>(
				(m_encodeStampsUs.size() - 1) * 1000000 / spanUs, 5, 120));
		}
	}
	// Histeresis: reconfigura solo si la cadencia varía >30% del nominal
	// configurado y han pasado ≥2 s desde la última reconfiguración (evita
	// palpitación de reset/IDR).
	const bool cadenceChanged = m_encoderNominalFps != 0
		&& nowUs - m_encoderEpochUs >= 2000000
		&& (cadenceFps * 10 > m_encoderNominalFps * 13
			|| cadenceFps * 13 < m_encoderNominalFps * 10);
	const uint8_t fps = static_cast<uint8_t>(std::min<uint64_t>(cadenceFps, 120u)); // metadata del rate-control; sin pacing (CR-029)
	const bool hardwareRequested = m_hardwareAccelerationEnabled.load();
	const bool encoderExists = m_videoEncoder || m_hardwareVideoEncoder;
	// CR-036: la cadencia medida ya no reinicia el encoder (solo metadata de rate-control).
	const bool encoderConfigChanged = m_encoderWidth != width || m_encoderHeight != height
		|| m_encoderBitrateKbps != bitrateKbps;
	const bool hardwarePreferenceChanged = hardwareRequested != m_encoderHardwarePreference;
	if (!encoderExists || encoderConfigChanged || hardwarePreferenceChanged)
	{
		if (m_hardwareVideoEncoder)
		{
			delete static_cast<TwinpadFfmpegEncoder*>(m_hardwareVideoEncoder);
			m_hardwareVideoEncoder = nullptr;
		}
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
			auto hardwareEncoder = CreateHardwareH264Encoder(width, height, fps, bitrateKbps);
			if (hardwareEncoder)
			{
				m_hardwareVideoEncoder = hardwareEncoder.release();
				m_encoderHardwareActive = true;
				cemuLog_log(LogType::Force, "[TwinpadVideo] encoder=hardware backend={}",
					static_cast<TwinpadFfmpegEncoder*>(m_hardwareVideoEncoder)->name);
			}
			else
			{
				cemuLog_log(LogType::Force, "[TwinpadVideo] hardware encoder unavailable; falling back to OpenH264 CPU");
			}
		}
		if (!m_encoderHardwareActive)
		{
			m_videoEncoder = CreateOpenH264Encoder(width, height, fps, bitrateKbps);
			if (!m_videoEncoder)
				return false;
			cemuLog_log(LogType::Force, "[TwinpadVideo] encoder=OpenH264 CPU");
		}	m_encoderWidth = width;
	m_encoderHeight = height;
	m_encoderBitrateKbps = bitrateKbps;
	m_encoderNominalFps = fps;
	m_encoderFrameIndex = 0;
	m_forceNextIdr = true;
	m_lastIdrTimestampUs = 0;
	m_encoderEpochUs = nowUs;
	}

	// CR-043: conversión RGB8→YUV acelerada (SSSE3 con fallback escalar
	// idéntico bit a bit). Encode hardware consume el buffer NV12 directo;
	// OpenH264 sigue recibiendo I420.
	const auto yuvStarted = std::chrono::steady_clock::now();
	const size_t ySize = static_cast<size_t>(width) * height;
	const bool hardwareAttempted = m_encoderHardwareActive;
	ConvertRgbToYuv(rgb.data(), width, height, hardwareAttempted, m_yuvBuffer);
	RecordVideoMetric(TwinpadVideoMetricStage::I420_CONVERT,
		std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - yuvStarted).count());
	uint8_t* const yPlane = m_yuvBuffer.data();
	uint8_t* const uPlane = yPlane + ySize;
	uint8_t* const vPlane = uPlane + ySize / 4;

	output.annexB.clear();
	bool forceIdr = m_forceNextIdr || nowUs - m_lastIdrTimestampUs >= kIdrIntervalMs * 1000;
	auto fallbackToOpenH264 = [&]() {
		delete static_cast<TwinpadFfmpegEncoder*>(m_hardwareVideoEncoder);
		m_hardwareVideoEncoder = nullptr;
		m_encoderHardwareActive = false;
		m_videoEncoder = CreateOpenH264Encoder(width, height, fps, bitrateKbps);
		if (!m_videoEncoder)
			return false;
		output.annexB.clear();
		m_forceNextIdr = true;
		m_lastIdrTimestampUs = 0;
		m_encoderEpochUs = nowUs;
		m_encoderFrameIndex = 0;
		forceIdr = true;
		cemuLog_log(LogType::Force, "[TwinpadVideo] hardware encoder violated the H.264 contract; falling back to OpenH264 CPU");
		return true;
	};
	if (m_encoderHardwareActive)
	{
		const auto hardwareStarted = std::chrono::steady_clock::now();
		auto* encoder = static_cast<TwinpadFfmpegEncoder*>(m_hardwareVideoEncoder);
		// CR-043: se entrega el buffer NV12 ya consolidado (Y + UV intercalado).
		const bool hardwareEncoded = encoder->Encode(yPlane, uPlane, width, height,
			static_cast<int64_t>(m_encoderFrameIndex), forceIdr, output.annexB);
		// VAAPI con pipeline interno devuelve EAGAIN: no es fallo, la AU se emite
		// en ticks posteriores (se avisa solo la primera vez y se conserva el backend).
		if (!hardwareEncoded && encoder->bufferedOutput)
		{
			static std::atomic_bool warnedBufferedOnce{false};
			if (!warnedBufferedOnce.exchange(true))
				cemuLog_log(LogType::Force, "[TwinpadVideo] hardware backend {} buffers frames (EAGAIN); AUs will arrive on later ticks", encoder->name);
			return false;
		}
		if (!hardwareEncoded)
			cemuLog_log(LogType::Force, "[TwinpadVideo] hardware backend {} did not produce an output AU on frame {} (idr_forced={})",
				encoder->name, m_encoderFrameIndex, forceIdr ? 1 : 0);
		if (!hardwareEncoded)
		{
			if (!fallbackToOpenH264())
				return false;
		}
		else
		{
			RecordVideoMetric(TwinpadVideoMetricStage::HARDWARE_ENCODER,
				std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - hardwareStarted).count());
			output.idr = ContainsIdrNal(output.annexB);
			if (forceIdr && (!output.idr
				|| !ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264SpsNalType)
				|| !ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264PpsNalType)
				|| !HasH264ContractProfile(output.annexB)))
			{
				if (!output.idr)
					cemuLog_log(LogType::Force, "[TwinpadVideo] hardware AE fell back: no IDR NAL in forced keyframe from {}", encoder->name);
				else if (!ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264SpsNalType))
					cemuLog_log(LogType::Force, "[TwinpadVideo] hardware AE fell back: forced keyframe from {} lacks SPS", encoder->name);
				else if (!ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264PpsNalType))
					cemuLog_log(LogType::Force, "[TwinpadVideo] hardware AE fell back: forced keyframe from {} lacks PPS", encoder->name);
				else
				{
					const auto sps = FirstH264Nal(output.annexB, kH264SpsNalType);
					if (sps.size() >= 4)
						cemuLog_log(LogType::Force, "[TwinpadVideo] hardware AE fell back: forced SPS from {} reads profile_idc={} constraint_flags=0x{:02x} level_idc={} (contract: profile 66/77, level ≤42)",
							encoder->name, static_cast<unsigned>(sps[1]), static_cast<unsigned>(sps[2]), static_cast<unsigned>(sps[3]));
					else
						cemuLog_log(LogType::Force, "[TwinpadVideo] hardware AE fell back: forced SPS from {} is too short ({} bytes) for contract parsing",
							encoder->name, sps.size());
				}
				if (!fallbackToOpenH264())
					return false;
			}
		}
		if (!hardwareEncoded || !m_encoderHardwareActive)
		{
			// CR-043: si el frame entrante era NV12 (hardwareAttempted) y caímos a
			// OpenH264, reconvertir a I420 en el mismo buffer antes del encode CPU.
			if (hardwareAttempted && !m_encoderHardwareActive)
				ConvertRgbToYuv(rgb.data(), width, height, false, m_yuvBuffer);
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
			if (output.idr && (!ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264SpsNalType)
				|| !ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264PpsNalType)
				|| !HasH264ContractProfile(output.annexB)))
				return false;
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
		if (output.idr && (!ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264SpsNalType)
			|| !ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264PpsNalType)
			|| !HasH264ContractProfile(output.annexB)))
			return false;
	}
	output.timestampUs = nowUs >= m_encoderEpochUs ? nowUs - m_encoderEpochUs : 0;
	if (forceIdr && (!output.idr || !ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264SpsNalType)
		|| !ContainsH264Nal(output.annexB.data(), output.annexB.size(), kH264PpsNalType)
		|| !HasH264ContractProfile(output.annexB)))
		return false;
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

std::string TwinpadClient::Base64Url(const uint8_t* data, size_t size)
{
	const size_t remainder = size % 3;
	const size_t outputSize = size / 3 * 4 + (remainder ? remainder + 1 : 0);
	std::string result(outputSize, '\0');
	char* output = result.data();
	size_t inputIndex = 0;
	size_t outputIndex = 0;
	while (inputIndex + 3 <= size)
	{
		const uint32_t block = (static_cast<uint32_t>(data[inputIndex]) << 16)
			| (static_cast<uint32_t>(data[inputIndex + 1]) << 8)
			| static_cast<uint32_t>(data[inputIndex + 2]);
		output[outputIndex++] = kBase64Alphabet[(block >> 18) & 63];
		output[outputIndex++] = kBase64Alphabet[(block >> 12) & 63];
		output[outputIndex++] = kBase64Alphabet[(block >> 6) & 63];
		output[outputIndex++] = kBase64Alphabet[block & 63];
		inputIndex += 3;
	}
	if (remainder != 0)
	{
		const uint32_t block = (static_cast<uint32_t>(data[inputIndex]) << 16)
			| (remainder == 2 ? static_cast<uint32_t>(data[inputIndex + 1]) << 8 : 0);
		output[outputIndex++] = kBase64Alphabet[(block >> 18) & 63];
		output[outputIndex++] = kBase64Alphabet[(block >> 12) & 63];
		if (remainder == 2)
			output[outputIndex] = kBase64Alphabet[(block >> 6) & 63];
	}
	return result;
}

bool TwinpadClient::GetCurrentQr(TwinpadQr& qr)
{
#if BOOST_OS_WINDOWS
	static const bool winsockReady = [] { WSADATA data{}; return WSAStartup(MAKEWORD(2, 2), &data) == 0; }();
	if (!winsockReady) return false;
#endif
	const SocketHandle socketHandle = socket(AF_INET, SOCK_STREAM, 0);
	if (socketHandle < 0) return false;
	sockaddr_in endpoint{};
	endpoint.sin_family = AF_INET;
	endpoint.sin_port = htons(kQrIpcPort);
	inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
	if (connect(socketHandle, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0)
	{
		CloseSocket(socketHandle);
		return false;
	}
#if BOOST_OS_WINDOWS
	DWORD timeout = 2000;
	setsockopt(socketHandle, SOL_SOCKET, SO_RCVTIMEO, reinterpret_cast<const char*>(&timeout), sizeof(timeout));
#else
	timeval timeout{2, 0};
	setsockopt(socketHandle, SOL_SOCKET, SO_RCVTIMEO, &timeout, sizeof(timeout));
#endif
	const std::string request = "GET_QR\n";
#if BOOST_OS_WINDOWS
	const int requestSent = ::send(socketHandle, request.data(), static_cast<int>(request.size()), 0);
#else
	const ssize_t requestSent = ::send(socketHandle, request.data(), request.size(), MSG_NOSIGNAL);
#endif
	if (requestSent != static_cast<decltype(requestSent)>(request.size()))
	{
		CloseSocket(socketHandle);
		return false;
	}
	std::string response;
	char buffer[2048];
	for (;;)
	{
#if BOOST_OS_WINDOWS
		const int count = ::recv(socketHandle, buffer, sizeof(buffer), 0);
#else
		const ssize_t count = ::recv(socketHandle, buffer, sizeof(buffer), 0);
#endif
		if (count <= 0) break;
		response.append(buffer, static_cast<size_t>(count));
		if (response.size() > kMaxQrResponseBytes)
		{
			CloseSocket(socketHandle);
			return false;
		}
		if (response.find('\n') != std::string::npos) break;
	}
	CloseSocket(socketHandle);
	const size_t lineEnd = response.find('\n');
	if (lineEnd == std::string::npos) return false;
	rapidjson::Document document;
	document.Parse(response.data(), lineEnd);
	if (document.HasParseError() || !document.IsObject() || !document.HasMember("uri")
		|| !document["uri"].IsString() || !document.HasMember("modules") || !document["modules"].IsArray()
		|| !document.HasMember("cemu_live") || !document["cemu_live"].IsBool())
		return false;
	TwinpadQr parsed;
	parsed.uri = document["uri"].GetString();
	parsed.cemuLive = document["cemu_live"].GetBool();
	const auto& rows = document["modules"];
	if (rows.Empty() || rows.Size() > 177) return false;
	const rapidjson::SizeType width = rows[0].IsString() ? rows[0].GetStringLength() : 0;
	if (width == 0 || width > 177) return false;
	parsed.modules.reserve(rows.Size());
	for (const auto& row : rows.GetArray())
	{
		if (!row.IsString() || row.GetStringLength() != width) return false;
		std::string modules(row.GetString(), row.GetStringLength());
		if (modules.find_first_not_of("01") != std::string::npos) return false;
		parsed.modules.push_back(std::move(modules));
	}
	qr = std::move(parsed);
	return true;
}

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

#ifdef TWINPAD_E2E_TEST_HOOK
namespace
{
void StartE2eInputProbe(std::string_view line)
{
	if (line.size() > kE2eRequestLimit || !line.starts_with("TEST-058 "))
		return;
	std::string request(line);
	request.push_back('\n');
	std::thread([request = std::move(request)] {
		const SocketHandle listener = socket(AF_INET, SOCK_STREAM, 0);
		if (listener < 0)
			return;
		int reuse = 1;
		setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
		sockaddr_in address{};
		address.sin_family = AF_INET;
		address.sin_port = htons(kE2eCaptureIpcPort);
		address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		if (bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 || listen(listener, 1) != 0)
		{
			CloseSocket(listener);
			return;
		}
		fd_set readable;
		FD_ZERO(&readable);
		FD_SET(listener, &readable);
		timeval timeout{10, 0};
#if BOOST_OS_WINDOWS
		const int ready = select(0, &readable, nullptr, nullptr, &timeout);
#else
		const int ready = select(listener + 1, &readable, nullptr, nullptr, &timeout);
#endif
		if (ready > 0)
		{
			const SocketHandle client = accept(listener, nullptr, nullptr);
			if (client >= 0)
			{
				char input[kE2eRequestLimit + 1]{};
#if BOOST_OS_WINDOWS
				const int size = recv(client, input, static_cast<int>(sizeof(input) - 1), 0);
#else
				const ssize_t size = recv(client, input, sizeof(input) - 1, 0);
#endif
				if (size > 0 && std::string_view(input, static_cast<size_t>(size)) == request)
				{
					const auto state = TwinpadClient::Instance().GetInputState();
					char response[256]{};
					const int responseSize = std::snprintf(response, sizeof(response),
						"{\"type\":\"test_probe\",\"seq\":%u,\"touch_x\":%u,\"touch_y\":%u,\"touch_pressed\":%s}\n",
						state.seq, state.touchX, state.touchY, state.touchPressed ? "true" : "false");
					const int32_t probeSeq = static_cast<int32_t>(gE2eInputProbeSeq.fetch_add(1));
					const std::string probeReport = "{\"type\":\"input_report\",\"seq\":"
						+ std::to_string(probeSeq) + ",\"stick_lx\":0,\"stick_ly\":0,\"stick_rx\":0,\"stick_ry\":0,"
						+ "\"dpad\":8,\"buttons\":0,\"trigger_l2\":0,\"trigger_r2\":0,\"touch_x\":"
						+ std::to_string(state.touchX) + ",\"touch_y\":" + std::to_string(state.touchY)
						+ ",\"touch_pressed\":" + (state.touchPressed ? "true" : "false") + "}";
					if (responseSize > 0 && SendLine(m_primarySocket, probeReport))
						::send(client, response, static_cast<size_t>(responseSize), 0);
				}
				CloseSocket(client);
			}
		}
		CloseSocket(listener);
	}).detach();
}
}
#endif

void TwinpadClient::RequestConnection()
{
	if (!m_titleActive.load() || !CafeSystem::IsTitleRunning())
		return;
	m_connectionRequested.store(true);
	m_captureGamePad.store(true);

	if (!m_running.load())
		Start(true);
}

void TwinpadClient::CancelConnectionRequest()
{
	std::scoped_lock lifecycleLock(m_lifecycleMutex);
	if (!m_titleActive.load())
		return;
	m_connectionRequested.store(false);
	m_captureGamePad.store(false);
	m_connected.store(false);

	{
		std::scoped_lock lock(m_primarySocketMutex);
		if (m_primarySocket >= 0)
		{
#if BOOST_OS_WINDOWS
			shutdown(static_cast<SocketHandle>(m_primarySocket), SD_BOTH);
#else
			shutdown(static_cast<SocketHandle>(m_primarySocket), SHUT_RDWR);
#endif
		}
	}
	{
		std::scoped_lock lock(m_audioSocketMutex);
		if (m_audioSocket >= 0)
		{
#if BOOST_OS_WINDOWS
			shutdown(static_cast<SocketHandle>(m_audioSocket), SD_BOTH);
#else
			shutdown(static_cast<SocketHandle>(m_audioSocket), SHUT_RDWR);
#endif
		}
	}
	{
		std::scoped_lock lock(m_inputMutex);
		m_inputState = TwinpadInputState{};
		m_gyroMonitorState = TwinpadInputState{};
		m_fusedMotionState = TwinpadFusedMotionState{};
		m_lastMotionReceived = {};
		m_hasFreshMotionInput = false;
		m_hasMotionTimestamp = false;
		m_lastMotionTimestampUs = 0;
		m_hasInputSeq = false;
		m_hasGyroMonitorState = false;
	}
}

void TwinpadClient::InputLoop()
{
	auto nextConnectionAttempt = std::chrono::steady_clock::now();
	bool titleWasActive = false;
	while (m_running.load())
	{
		if (!m_titleActive.load() || !m_connectionRequested.load() || !CafeSystem::IsTitleRunning())
		{
			const bool disconnectRequested = !m_titleActive.load() || !CafeSystem::IsTitleRunning();
			if (titleWasActive || disconnectRequested)
			{
				m_connected.store(false);
				{
					std::scoped_lock lock(m_primarySocketMutex);
					if (m_primarySocket >= 0)
						CloseSocket(static_cast<SocketHandle>(m_primarySocket));
					m_primarySocket = -1;
				}
				{
					std::scoped_lock lock(m_audioSocketMutex);
					if (m_audioSocket >= 0)
						CloseSocket(static_cast<SocketHandle>(m_audioSocket));
					m_audioSocket = -1;
				}
				m_receiveBuffer.clear();

				{
					std::scoped_lock lock(m_audioQueueMutex);
					m_audioQueue.clear();
				}
				{
					std::scoped_lock lock(m_inputMutex);
					m_inputState = TwinpadInputState{};
					m_gyroMonitorState = TwinpadInputState{};
					m_fusedMotionState = TwinpadFusedMotionState{};
					m_lastMotionReceived = {};
				m_hasFreshMotionInput = false;
				m_hasMotionTimestamp = false;
				m_lastMotionTimestampUs = 0;
				m_hasInputSeq = false;
				m_hasGyroMonitorState = false;
				}
			}
			if (!m_titleActive.load() || !m_connectionRequested.load())
				titleWasActive = false;
			std::this_thread::sleep_for(std::chrono::milliseconds(50));
			nextConnectionAttempt = std::chrono::steady_clock::now();
			continue;
		}
		titleWasActive = true;
		if (!m_connected.load())
		{
			const auto now = std::chrono::steady_clock::now();
			if (now < nextConnectionAttempt)
			{
				std::this_thread::sleep_for(std::chrono::milliseconds(50));
				continue;
			}
			{
				std::scoped_lock lock(m_primarySocketMutex);
				if (m_primarySocket >= 0)
					CloseSocket(static_cast<SocketHandle>(m_primarySocket));
				m_primarySocket = -1;
			}
			{
				std::scoped_lock lock(m_audioSocketMutex);
				if (m_audioSocket >= 0)
					CloseSocket(static_cast<SocketHandle>(m_audioSocket));
				m_audioSocket = -1;
			}
			const bool connected = ConnectPrimary() && m_titleActive.load() && ConnectAudio();
			if (connected && m_titleActive.load() && m_connectionRequested.load())
			{
				m_frameNumber.store(0);
				m_audioSequence.store(0);
				ResetVideoEncoder();
				m_receiveBuffer.clear();
				{
					std::scoped_lock lock(m_inputMutex);
				m_inputState = TwinpadInputState{};
				m_gyroMonitorState = TwinpadInputState{};
				m_fusedMotionState = TwinpadFusedMotionState{};
				m_lastMotionReceived = {};
				m_hasFreshMotionInput = false;
				m_hasMotionTimestamp = false;
				m_lastMotionTimestampUs = 0;
				m_hasInputSeq = false;
				m_hasGyroMonitorState = false;
				}

				{
					std::scoped_lock lock(m_audioQueueMutex);
					m_audioQueue.clear();
				}
				m_connected.store(true);
				nextConnectionAttempt = std::chrono::steady_clock::now()
					+ std::chrono::milliseconds(kRetryIntervalMs);
			}
			else
			{
				{
					std::scoped_lock lock(m_primarySocketMutex);
					if (m_primarySocket >= 0)
						CloseSocket(static_cast<SocketHandle>(m_primarySocket));
					m_primarySocket = -1;
				}
				{
					std::scoped_lock lock(m_audioSocketMutex);
					if (m_audioSocket >= 0)
						CloseSocket(static_cast<SocketHandle>(m_audioSocket));
					m_audioSocket = -1;
				}
				nextConnectionAttempt = std::chrono::steady_clock::now()
					+ std::chrono::milliseconds(kRetryIntervalMs);
				continue;
			}
		}
		std::intptr_t socket = -1;
		{
			std::scoped_lock lock(m_primarySocketMutex);
			socket = m_primarySocket;
		}
		if (socket < 0)
		{
			m_connected.store(false);
			nextConnectionAttempt = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(kRetryIntervalMs);
			continue;
		}
		char buffer[8192];
#if BOOST_OS_WINDOWS
		const int count = ::recv(static_cast<SocketHandle>(socket), buffer, sizeof(buffer) - 1, 0);
#else
		const ssize_t count = ::recv(static_cast<SocketHandle>(socket), buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
#endif
		if (count > 0)
		{
			m_receiveBuffer.append(buffer, static_cast<size_t>(count));
#ifdef TWINPAD_E2E_TEST_HOOK
			const auto probeEnd = m_receiveBuffer.find("TEST-058 ");
			if (probeEnd != std::string::npos)
			{
				const auto lineEnd = m_receiveBuffer.find('\n', probeEnd);
				if (lineEnd != std::string::npos)
				{
					StartE2eInputProbe(std::string_view(m_receiveBuffer).substr(probeEnd, lineEnd - probeEnd));
					m_receiveBuffer.erase(probeEnd, lineEnd - probeEnd + 1);
				}
			}
#endif
			if (m_receiveBuffer.size() > 64 * 1024 && m_receiveBuffer.find('\n') == std::string::npos)
				m_receiveBuffer.clear();
			size_t lineEnd = m_receiveBuffer.find('\n');
			while (lineEnd != std::string::npos)
			{
				rapidjson::Document document;
				document.Parse(m_receiveBuffer.data(), lineEnd);
				if (!document.HasParseError())
				{
					if (document.IsObject() && document.HasMember("type") && document["type"].IsString()
						&& std::string_view(document["type"].GetString()) == "video_config")
					{
						if (document.HasMember("bitrate_kbps") && document["bitrate_kbps"].IsUint())
							SetCaptureBitrate(std::clamp(document["bitrate_kbps"].GetUint(), 1000u, 30000u)); // CR-027; fps ignorado (CR-029)
						// CR-030: resolución de emisión del GamePad (DRC); sin w/h se mantiene nativo.
						if (document.HasMember("w") && document["w"].IsUint()
							&& document.HasMember("h") && document["h"].IsUint())
							SetCaptureResolution(document["w"].GetUint(), document["h"].GetUint());
					}
					else if (document.IsObject() && document.HasMember("type") && document["type"].IsString()
						&& std::string_view(document["type"].GetString()) == "camera_frame")
					{
						// CR-034: camera_frame H.264 320x240 del teléfono; se decodifica aquí.
						TwinpadCameraDecoder::Instance().DecodeLine(std::string(m_receiveBuffer.data(), lineEnd));
					}
					else if (document.IsObject() && document.HasMember("type") && document["type"].IsString()
						&& std::string_view(document["type"].GetString()) == "mic_chunk")
					{
						// CR-035: PCM s16le mono 32 kHz del teléfono; entra a la cola
						// drop-old que consume el HLE `mic` bajo demanda.
						FeedMicChunk(std::string_view(m_receiveBuffer.data(), lineEnd));
					}
					const auto report = ParseInputReport(document);
					if (report)
					{
						std::scoped_lock lock(m_inputMutex);
						if (!m_hasInputSeq || report->seq > m_inputState.seq)
						{
							m_inputState = *report;
							m_gyroMonitorState = *report;
							m_hasGyroMonitorState = report->motionValid;
							m_hasInputSeq = true;

							if (report->motionValid
								&& (!m_hasMotionTimestamp || report->motionTimestampUs > m_lastMotionTimestampUs))
							{
								m_lastMotionTimestampUs = report->motionTimestampUs;
								m_hasMotionTimestamp = true;
								m_lastMotionReceived = std::chrono::steady_clock::now();
								m_hasFreshMotionInput = true;
							}
							else if (!report->motionValid)
							{
								m_hasFreshMotionInput = false;
								m_hasMotionTimestamp = false;
								m_lastMotionTimestampUs = 0;
								m_lastMotionReceived = {};
								m_fusedMotionState = TwinpadFusedMotionState{};
							}

							if (report->motionValid && report->seq % kGyroLogReportInterval == 0)
								cemuLog_log(LogType::Force,
									"[TwinpadGyro] stage=cemu_rx seq={} timestamp_us={} accel=({:.5f},{:.5f},{:.5f}) gyro=({:.5f},{:.5f},{:.5f})",
									report->seq, report->motionTimestampUs,
									report->accel[0], report->accel[1], report->accel[2],
									report->gyro[0], report->gyro[1], report->gyro[2]);
						}
					}
				}
				m_receiveBuffer.erase(0, lineEnd + 1);
				lineEnd = m_receiveBuffer.find('\n');
			}
		}
#if BOOST_OS_WINDOWS
		else if (count < 0 && WSAGetLastError() != WSAEWOULDBLOCK)
#else
		else if (count < 0 && errno != EAGAIN && errno != EWOULDBLOCK)
#endif
		{
			m_receiveBuffer.clear();
			m_connected.store(false);
			{
				std::scoped_lock lock(m_primarySocketMutex);
				if (m_primarySocket >= 0)
					CloseSocket(static_cast<SocketHandle>(m_primarySocket));
				m_primarySocket = -1;
			}
			{
				std::scoped_lock lock(m_audioSocketMutex);
				if (m_audioSocket >= 0)
					CloseSocket(static_cast<SocketHandle>(m_audioSocket));
				m_audioSocket = -1;
			}
			{
				std::scoped_lock lock(m_inputMutex);
				m_inputState = TwinpadInputState{};
				m_gyroMonitorState = TwinpadInputState{};
				m_fusedMotionState = TwinpadFusedMotionState{};
				m_lastMotionReceived = {};
				m_hasFreshMotionInput = false;
				m_hasMotionTimestamp = false;
				m_lastMotionTimestampUs = 0;
				m_hasInputSeq = false;
					m_hasGyroMonitorState = false;
				}

				{
					std::scoped_lock lock(m_audioQueueMutex);
					m_audioQueue.clear();
			}
			nextConnectionAttempt = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(kRetryIntervalMs);
		}
		else if (count == 0)
		{
			m_receiveBuffer.clear();
			m_connected.store(false);
			{
				std::scoped_lock lock(m_primarySocketMutex);
				if (m_primarySocket >= 0)
					CloseSocket(static_cast<SocketHandle>(m_primarySocket));
				m_primarySocket = -1;
			}
			{
				std::scoped_lock lock(m_audioSocketMutex);
				if (m_audioSocket >= 0)
					CloseSocket(static_cast<SocketHandle>(m_audioSocket));
				m_audioSocket = -1;
			}
			{
				std::scoped_lock lock(m_inputMutex);
				m_inputState = TwinpadInputState{};
				m_gyroMonitorState = TwinpadInputState{};
				m_fusedMotionState = TwinpadFusedMotionState{};
				m_lastMotionReceived = {};
				m_hasFreshMotionInput = false;
				m_hasMotionTimestamp = false;
				m_lastMotionTimestampUs = 0;
				m_hasInputSeq = false;
					m_hasGyroMonitorState = false;
				}

				{
					std::scoped_lock lock(m_audioQueueMutex);
					m_audioQueue.clear();
			}
			nextConnectionAttempt = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(kRetryIntervalMs);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
	}
}

void TwinpadClient::RecordVideoMetric(TwinpadVideoMetricStage stage, uint64_t elapsedNanoseconds)
{
	const size_t index = static_cast<size_t>(stage);
	m_videoMetricCounts[index].fetch_add(1, std::memory_order_relaxed);
	m_videoMetricNanoseconds[index].fetch_add(elapsedNanoseconds, std::memory_order_relaxed);
	uint64_t maximum = m_videoMetricMaxNanoseconds[index].load(std::memory_order_relaxed);
	while (maximum < elapsedNanoseconds
		&& !m_videoMetricMaxNanoseconds[index].compare_exchange_weak(maximum, elapsedNanoseconds, std::memory_order_relaxed))
	{
	}
}

void TwinpadClient::RecordVideoEvent(TwinpadVideoMetricEvent event)
{
	m_videoMetricEvents[static_cast<size_t>(event)].fetch_add(1, std::memory_order_relaxed);
}

void TwinpadClient::FlushVideoMetricsIfDue()
{
	const uint64_t nowMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	uint64_t previousMs = m_lastVideoMetricLogMs.load(std::memory_order_relaxed);
	if (nowMs - previousMs < kVideoMetricLogIntervalMs
		|| !m_lastVideoMetricLogMs.compare_exchange_strong(previousMs, nowMs, std::memory_order_relaxed))
		return;

	static constexpr const char* stageNames[] = {
		"readback_copy", "readback_ready", "pixel_convert", "scale", "i420_convert",
		"hardware_encoder", "openh264", "encode_pipeline", "base64_json", "ipc_send",
	};
	static constexpr const char* eventNames[] = {
		"drc_callback", "capture_accepted", "readback_queued", "readback_completed",
		"readback_slot_busy", "frame_queued", "frame_encoded", "frame_sent",
	};
	std::string summary = "[TwinpadPerf] window_ms=" + std::to_string(kVideoMetricLogIntervalMs);
	for (size_t index = 0; index < m_videoMetricCounts.size(); ++index)
	{
		const uint64_t count = m_videoMetricCounts[index].exchange(0, std::memory_order_relaxed);
		const uint64_t totalNs = m_videoMetricNanoseconds[index].exchange(0, std::memory_order_relaxed);
		const uint64_t maxNs = m_videoMetricMaxNanoseconds[index].exchange(0, std::memory_order_relaxed);
		if (count != 0)
			summary += " " + std::string(stageNames[index]) + "_n=" + std::to_string(count)
				+ "_avg_us=" + std::to_string(totalNs / count / 1000)
				+ "_max_us=" + std::to_string(maxNs / 1000);
	}
	for (size_t index = 0; index < m_videoMetricEvents.size(); ++index)
	{
		const uint64_t count = m_videoMetricEvents[index].exchange(0, std::memory_order_relaxed);
		if (count != 0)
			summary += " " + std::string(eventNames[index]) + "=" + std::to_string(count);
	}
	cemuLog_log(LogType::Force, "{}", summary);
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

void TwinpadClient::PublishRumble(const uint8_t* pattern, uint8_t length)
{
	if (!m_connected.load())
		return;
	static constexpr char kBase64Url[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
	std::string encoded;
	for (size_t i = 0; i < length; i += 3)
	{
		const uint32_t b0 = pattern[i];
		const uint32_t b1 = i + 1 < length ? pattern[i + 1] : 0;
		const uint32_t b2 = i + 2 < length ? pattern[i + 2] : 0;
		const uint32_t triple = (b0 << 16) | (b1 << 8) | b2;
		encoded.push_back(kBase64Url[(triple >> 18) & 63]);
		encoded.push_back(kBase64Url[(triple >> 12) & 63]);
		if (i + 1 < length)
			encoded.push_back(kBase64Url[(triple >> 6) & 63]);
		if (i + 2 < length)
			encoded.push_back(kBase64Url[triple & 63]);
	}
	const std::string line = "{\"type\":\"rumble\",\"length\":" + std::to_string(length) + ",\"data\":\"" + encoded + "\"}";
	std::scoped_lock lock(m_primarySocketMutex);
	if (!m_connected.load() || m_primarySocket < 0)
		return;
	if (!SendLine(m_primarySocket, line))
		m_connected.store(false);
}

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

namespace
{
// CR-035: decodificador base64url sin relleno (mismo contrato que core;
// duplicado de TwinpadCameraDecoder porque el codec H.264 no aplica aquí).
int DecodeBase64UrlCharMic(char c)
{
	if (c >= 'A' && c <= 'Z') return c - 'A';
	if (c >= 'a' && c <= 'z') return c - 'a' + 26;
	if (c >= '0' && c <= '9') return c - '0' + 52;
	if (c == '-') return 62;
	if (c == '_') return 63;
	return -1;
}

bool DecodeBase64UrlMic(const char* text, size_t size, std::vector<uint8_t>& out)
{
	out.clear();
	if (size == 0 || size % 4 == 1)
		return false;
	uint32_t buffer = 0;
	int bits = 0;
	for (size_t i = 0; i < size; ++i)
	{
		const int value = DecodeBase64UrlCharMic(text[i]);
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
	return true;
}
} // namespace

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
	m_micDemandOn.store(on);
	std::scoped_lock lock(m_primarySocketMutex);
	if (!m_connected.load() || m_primarySocket < 0)
		return;
	if (!SendLine(m_primarySocket, MicRequestLine(on)))
		m_connected.store(false);
}

bool TwinpadClient::FeedMicChunk(std::string_view line)
{
	// CR-035: `mic_chunk` estricto — samples 32..32000 (1 ms..1 s @ 32 kHz)
	// y 2 bytes por muestra s16le; lo incoherente se ignora (E4).
	rapidjson::Document document;
	document.Parse(line.data(), line.size());
	if (document.HasParseError() || !document.IsObject())
		return false;
	if (!document.HasMember("type") || !document["type"].IsString()
		|| std::string_view(document["type"].GetString()) != "mic_chunk")
		return false;
	if (!document.HasMember("samples") || !document["samples"].IsUint())
		return false;
	const uint32_t samples = document["samples"].GetUint();
	if (samples < 32 || samples > 32000)
		return false;
	if (!document.HasMember("data") || !document["data"].IsString())
		return false;
	const char* base64Data = document["data"].GetString();
	std::vector<uint8_t> decoded;
	if (!DecodeBase64UrlMic(base64Data, document["data"].GetStringLength(), decoded))
		return false;
	if (decoded.size() != static_cast<size_t>(samples) * 2)
		return false;
	std::deque<int16_t> chunk;
	for (size_t i = 0; i < decoded.size(); i += 2)
		chunk.push_back(static_cast<int16_t>(static_cast<uint16_t>(decoded[i]) | (static_cast<uint16_t>(decoded[i + 1]) << 8)));
	{
		// Drop-old: si la cola superaría 1 s de audio, se descarta lo más antiguo.
		std::scoped_lock lock(m_micSamplesMutex);
		const size_t overflow = m_micSamples.size() + chunk.size() - kMicQueueMaxSamples;
		if (overflow > 0)
			m_micSamples.erase(m_micSamples.begin(), m_micSamples.begin() + static_cast<std::ptrdiff_t>(std::min(overflow, m_micSamples.size())));
		m_micSamples.insert(m_micSamples.end(), chunk.begin(), chunk.end());
	}
	m_lastMicChunkMs.store(static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count()));
	return true;
}

bool TwinpadClient::ConsumeMicSamples(int16_t* dst, size_t count)
{
	if (!dst || count == 0 || !m_micDemandOn.load())
		return false;
	const auto nowMs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	// CR-035: sin chunks frescos el HLE vuelve a sus caminos existentes
	// (dispositivo de entrada del PC / blow mic / silencio).
	const uint64_t last = m_lastMicChunkMs.load();
	if (nowMs > last + kMicFreshIntervalMs)
		return false;
	std::scoped_lock lock(m_micSamplesMutex);
	if (m_micSamples.size() < count)
		return false;
	for (size_t i = 0; i < count; ++i)
	{
		dst[i] = m_micSamples.front();
		m_micSamples.pop_front();
	}
	return true;
}

void TwinpadClient::FrameLoop()
{
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
		if (format == TwinpadPixelFormat::Rgb8Srgb)
		{
			std::transform(frameData.begin(), frameData.end(), frameData.begin(), SrgbToRgb);
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
					rgb[destination + channel] = applySrgb ? SrgbToRgb(value) : value;
				}
			}
			frameData = std::move(rgb);
		}
		RecordVideoMetric(TwinpadVideoMetricStage::PIXEL_CONVERT,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - pixelConvertStarted).count());
		// CR-030: aplicar la resolución de emisión del GamePad (DRC). Solo
		// reduce: si el target >= nativo se publica nativo (sin upscale).
		const uint32_t targetWidth = m_captureWidth.load(std::memory_order_relaxed);
		const uint32_t targetHeight = m_captureHeight.load(std::memory_order_relaxed);
		if (targetWidth > 0 && targetHeight > 0
			&& (targetWidth < static_cast<uint32_t>(width)
				|| targetHeight < static_cast<uint32_t>(height)))
		{
			const auto scaleStarted = std::chrono::steady_clock::now();
			frameData = ScaleRgb(frameData, width, height, static_cast<uint16_t>(targetWidth), static_cast<uint16_t>(targetHeight));
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
		const uint64_t frame = m_frameNumber.fetch_add(1);
		const auto base64JsonStarted = std::chrono::steady_clock::now();
		const std::string encodedData = Base64Url(encodedFrame.annexB.data(), encodedFrame.annexB.size());
		std::string line;
		line.reserve(encodedData.size() + 192);
		line = "{\"type\":\"game_frame\",\"gfn\":" + std::to_string(frame)
			+ ",\"w\":" + std::to_string(width) + ",\"h\":" + std::to_string(height)
			+ ",\"fmt\":\"h264\",\"ts_capture_us\":" + std::to_string(encodedFrame.timestampUs)
			+ ",\"idr\":" + (encodedFrame.idr ? "true" : "false")
			+ ",\"data\":\"" + encodedData + "\"}";
		RecordVideoMetric(TwinpadVideoMetricStage::BASE64_JSON,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - base64JsonStarted).count());
		const auto ipcSendStarted = std::chrono::steady_clock::now();
		std::scoped_lock lock(m_primarySocketMutex);
		if (!m_connected.load() || m_primarySocket < 0)
			continue;
		const uint64_t now = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
		if (!SendLine(m_primarySocket, line))
		{
			m_connected.store(false);
			continue;
		}
		RecordVideoMetric(TwinpadVideoMetricStage::IPC_SEND,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - ipcSendStarted).count());
		RecordVideoEvent(TwinpadVideoMetricEvent::FRAME_SENT);
		m_lastFrameSentMs.store(now);
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
				+ std::to_string(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count())
				+ ",\"data\":\"" + Base64Url(bytes + offset, amount) + "\"}";
			std::scoped_lock lock(m_audioSocketMutex);
			if (m_audioSocket < 0 || !SendLine(m_audioSocket, line))
			{
				m_connected.store(false);
				if (m_audioSocket >= 0)
					CloseSocket(static_cast<SocketHandle>(m_audioSocket));
				m_audioSocket = -1;
				break;
			}
		}
	}
}
