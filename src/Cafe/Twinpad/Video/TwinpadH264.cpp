#include "Cafe/Twinpad/Video/TwinpadH264.h"

#include "Common/precompiled.h"

#include <algorithm>
#include <array>
#include <cstring>
#include <filesystem>
#include <memory>
#include <string>

extern "C"
{
#include <libavcodec/avcodec.h>
#include <libavcodec/bsf.h>
#include <libavcodec/defs.h>
#include <libavcodec/packet.h>
#include <libavutil/avutil.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/opt.h>
}
#include <wels/codec_api.h>

#if BOOST_OS_LINUX
#include <unistd.h>
#endif

namespace twinpad_h264
{

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

// CR-040 verificacion: payload del primer NAL del tipo pedido (sin start code),
// vacio si no existe. Las cabeceras de SPS necesarios caben con holgura.
std::vector<uint8_t> FirstH264Nal(const std::vector<uint8_t>& annexB, uint8_t nalType)
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

bool ContainsIdrNal(const std::vector<uint8_t>& annexB)
{
	return ContainsH264Nal(annexB.data(), annexB.size(), kH264IdrNalType);
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

// CR-041: contrato del stream relajado — Baseline (66, con o sin constraint_set0)
// o Main (77), nivel <= 4.2, sin B-frames. Constrained Baseline 4.2
// (66 + set0 + 42) sigue siendo un subconjunto valido. Motivo: el driver
// radeonsi freeworld no expone Constrained Baseline y el encoder VAAPI rebota
// definitivamente con el contrato estricto (log: negotiated profile=578, result=-38).
// level_idc usa byte directo para todos los niveles estandar; 0 no es un nivel valido.
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

// --- Backend hardware (FFmpeg) ---------------------------------------------
// La struct interna conserva el encoding verbatim del cliente original; el
// PIMPL de TwinpadHardwareEncoder envuelve exactamente esa vida (CR-058).
struct TwinpadHardwareEncoder::Impl
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
	bool bufferedOutput{}; // el ultimo Encode() devolvio EAGAIN: el frame sigue en pipeline (saldra mas tarde)
	std::vector<uint8_t> annexBParameterSets;
	std::string name;

	~Impl()
	{
		av_bsf_free(&annexBFilter);
		av_packet_free(&packet);
		av_frame_free(&softwareFrame);
		av_frame_free(&frame);
		av_buffer_unref(&hwFrames);
		av_buffer_unref(&hwDevice);
		avcodec_free_context(&context);
	}

	bool Encode(const uint8_t* nv12Y, const uint8_t* nv12Chroma,
		int width, int height, int64_t frameIndex, bool forceIdr,
		std::vector<uint8_t>& output);
};

bool TwinpadHardwareEncoder::Impl::Encode(const uint8_t* nv12Y, const uint8_t* nv12Chroma,
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
	}
	av_packet_unref(packet);
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
		// (p. ej. VAAPI con asincronia): NO falla, emitira la AU mas tarde.
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

// CR-039: en Linux, la variante GNU/libre del driver VA-API de AMD no publica
// el entrypoint de encode H.264; el driver completo (RPM Fusion Free,
// mesa-va-drivers-freeworld) solo se resuelve si libva recibe
// LIBVA_DRIVERS_PATH. Tras un primer fallo de creacion del device, exporta
// el directorio freeworld (instalado o junto al ejecutable) y reintenta una vez.
static bool CreateVaapiHwDeviceForTwinpad(AVBufferRef** device)
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

static std::unique_ptr<TwinpadHardwareEncoder::Impl> CreateHardwareH264EncoderImpl(
	uint16_t width, uint16_t height, uint8_t fps, uint32_t bitrateKbps)
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
		auto encoder = std::make_unique<TwinpadHardwareEncoder::Impl>();
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
				deviceCreated = CreateVaapiHwDeviceForTwinpad(&encoder->hwDevice);
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
		context->max_b_frames = 0;
		// CR-041: VAAPI negocia el perfil disponible del driver (Baseline o Main en
		// radeonsi freeworld); los demas backends siguen pidiendo Constrained Baseline.
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
		}
		else if (encoder->name == "h264_vaapi")
		{
			av_dict_set(&options, "rc_mode", "CBR", 0);
			// CR-041: "constrained_baseline" rebota en el driver freeworld (no lo
			// expone: No usable encoding profile found). Sin la opcion el driver
			// negocia High (100), fuera del contrato. Pedimos Main (77): si el
			// driver no lo soporta, el descarte queda fail-loud en el log.
			av_dict_set(&options, "profile", "main", 0);
			// Minimizar el pipeline interno del encoder para que la IA salga
			// (casi) en el mismo tick que se envia el frame (latencia baja).
			av_dict_set(&options, "async_depth", "1", 0);
		}
		else if (encoder->name == "h264_amf")
		{
			av_dict_set(&options, "usage", "ultralowlatency", 0);
			av_dict_set(&options, "bf", "0", 0);
			av_dict_set(&options, "profile", "constrained_baseline", 0);
		}
		const int openResult = avcodec_open2(context, codec, &options);
		const bool optionsAccepted = av_dict_count(options) == 0;
		av_dict_free(&options);
		// CR-041: el candidato negociado debe caer en el contrato del stream
		// (Constrained Baseline 578, Baseline 66 o Main 77); solo los backends
		// no-VAAPI fuerzan Constrained Baseline desde la configuracion.
		const bool contractProfile = context->profile == AV_PROFILE_H264_CONSTRAINED_BASELINE
			|| context->profile == AV_PROFILE_H264_BASELINE
			|| context->profile == AV_PROFILE_H264_MAIN;
		if (openResult < 0 || !optionsAccepted
			|| !contractProfile
			|| context->level <= 0 || context->level > 42 || context->max_b_frames != 0)
		{
			// CR-040 verificacion: motivo exacto del descarte del candidato (CR-041 diagnostico de creacion).
			cemuLog_log(LogType::Force, "[TwinpadVideo] hardware candidate {} discarded at open: result={} options_left={} negotiated profile={} level={} b_frames={} (contract: profile 578/66/77, level <=42, b=0)",
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
			: extraDataSize == 0;
		if (avcConfiguration && (!hasParameterSets
			|| !HasH264ContractProfile(encoder->annexBParameterSets)))
		{
			const auto& parameterSets = encoder->annexBParameterSets;
			if (hasParameterSets && !parameterSets.empty())
			{
				const auto sps = FirstH264Nal(parameterSets, kH264SpsNalType);
				if (sps.size() >= 4)
					cemuLog_log(LogType::Force, "[TwinpadVideo] hardware candidate {} discarded: extradata SPS reads profile_idc={} constraint_flags=0x{:02x} level_idc={} (contract: profile 66/77, level <=42)",
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
		// del caller (zero-copy); la reserva propia quedo muerta.
		return encoder;
	}
	return nullptr;
}
// --- Fachada opaca TwinpadHardwareEncoder ----------------------------------

TwinpadHardwareEncoder::TwinpadHardwareEncoder(Impl* impl, std::string name, bool nv12, bool hardwareFrames)
	: m_impl(impl)
	, m_name(std::move(name))
	, m_nv12(nv12)
	, m_hardwareFrames(hardwareFrames)
{
	m_width = m_impl ? m_impl->context->width : 0;
	m_height = m_impl ? m_impl->context->height : 0;
	m_fps = m_impl ? static_cast<uint8_t>(m_impl->context->framerate.num) : 0;
	m_bitrateKbps = m_impl ? static_cast<uint32_t>(m_impl->context->bit_rate / 1000) : 0;
}

TwinpadHardwareEncoder::~TwinpadHardwareEncoder()
{
	delete m_impl;
}

bool TwinpadHardwareEncoder::IsOpen() const
{
	return m_impl != nullptr && m_impl->context != nullptr;
}

bool TwinpadHardwareEncoder::UsesNv12() const
{
	return m_nv12;
}

uint16_t TwinpadHardwareEncoder::GetWidth() const
{
	return m_width;
}

uint16_t TwinpadHardwareEncoder::GetHeight() const
{
	return m_height;
}

uint8_t TwinpadHardwareEncoder::GetFps() const
{
	return m_fps;
}

uint32_t TwinpadHardwareEncoder::GetBitrateKbps() const
{
	return m_bitrateKbps;
}

bool TwinpadHardwareEncoder::Encode(const uint8_t* plane0Y, const uint8_t* planeChroma, int width, int height,
	int64_t frameIndex, bool forceIdr, std::vector<uint8_t>& output)
{
	output.clear();
	if (!m_impl)
		return false;
	const bool ok = m_impl->Encode(plane0Y, planeChroma, width, height, frameIndex, forceIdr, output);
	m_bufferedOutput = m_impl->bufferedOutput;
	return ok;
}

std::unique_ptr<TwinpadHardwareEncoder> TwinpadHardwareEncoder::TryCreate(uint16_t width, uint16_t height,
	uint8_t fps, uint32_t bitrateKbps)
{
	auto impl = CreateHardwareH264EncoderImpl(width, height, fps, bitrateKbps);
	if (!impl)
		return nullptr;
	auto encoder = std::make_unique<TwinpadHardwareEncoder>(impl.get(), impl->name, impl->nv12, impl->hardwareFrames);
	impl.release(); // la clase pasa a ser duena del Impl
	return encoder;
}

// --- Backend OpenH264 (fallback CPU, transplante verbatim) ------------------

namespace
{

void* CreateOpenH264EncoderImpl(uint16_t width, uint16_t height, uint8_t fps, uint32_t bitrateKbps)
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

} // namespace

void* CreateOpenH264Encoder(uint16_t width, uint16_t height, uint8_t fps, uint32_t bitrateKbps)
{
	return CreateOpenH264EncoderImpl(width, height, fps, bitrateKbps);
}

void DestroyOpenH264Encoder(void* handle)
{
	if (handle != nullptr)
		WelsDestroySVCEncoder(static_cast<ISVCEncoder*>(handle));
}

} // namespace twinpad_h264
