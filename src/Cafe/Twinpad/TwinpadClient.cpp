#include "TwinpadClient.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <thread>
#include <cerrno>
#include <cstdio>
#include <optional>
#include <string_view>
#include <rapidjson/document.h>
#include <wels/codec_api.h>
#include "Cafe/CafeSystem.h"
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
constexpr uint32_t kOpenH264Bitrate = 8'000'000;
constexpr uint64_t kVideoMetricLogIntervalMs = 5'000;
constexpr uint32_t kOpenH264SliceCount = 4;
constexpr uint32_t kGyroLogReportInterval = 10;
constexpr char kBase64Alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

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

	if (document.HasMember("gyro"))
	{
		const auto& gyro = document["gyro"];
		if (!gyro.IsArray() || gyro.Size() != 3)
			return std::nullopt;
		int16_t* axes[] = {&state.yaw, &state.pitch, &state.roll};
		for (rapidjson::SizeType index = 0; index < gyro.Size(); ++index)
		{
			if (!gyro[index].IsInt())
				return std::nullopt;
			const int value = gyro[index].GetInt();
			if (value < -32768 || value > 32767)
				return std::nullopt;
			*axes[index] = static_cast<int16_t>(value);
		}
		state.gyroValid = true;
	}
	return state;	}
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

void TwinpadClient::ResetVideoEncoder()
{
	std::scoped_lock encoderLock(m_encoderMutex);
	if (m_videoEncoder)
	{
		auto* encoder = static_cast<ISVCEncoder*>(m_videoEncoder);
		encoder->Uninitialize();
		WelsDestroySVCEncoder(encoder);
		m_videoEncoder = nullptr;
	}
	m_encoderWidth = 0;
	m_encoderHeight = 0;
	m_encoderFps = 0;
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
	m_lastFrameQueuedUs.store(0);
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

	const uint8_t fps = std::clamp<uint8_t>(m_captureFps.load(), 1, 60);
	const uint64_t nowUs = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	std::scoped_lock lock(m_encoderMutex);
	if (!m_videoEncoder || m_encoderWidth != width || m_encoderHeight != height || m_encoderFps != fps)
	{
		if (m_videoEncoder)
		{
			auto* oldEncoder = static_cast<ISVCEncoder*>(m_videoEncoder);
			oldEncoder->Uninitialize();
			WelsDestroySVCEncoder(oldEncoder);
			m_videoEncoder = nullptr;
		}

		ISVCEncoder* encoder = nullptr;
		if (WelsCreateSVCEncoder(&encoder) != 0 || !encoder)
			return false;
		SEncParamExt params{};
		if (encoder->GetDefaultParams(&params) != 0)
		{
			WelsDestroySVCEncoder(encoder);
			return false;
		}
		params.iUsageType = SCREEN_CONTENT_REAL_TIME;
		params.iComplexityMode = LOW_COMPLEXITY;
		params.iPicWidth = width;
		params.iPicHeight = height;
		params.iTargetBitrate = kOpenH264Bitrate;
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
		params.sSpatialLayers[0].iSpatialBitrate = kOpenH264Bitrate;
		params.sSpatialLayers[0].iMaxSpatialBitrate = kOpenH264Bitrate;
		params.sSpatialLayers[0].uiProfileIdc = PRO_BASELINE;
		params.sSpatialLayers[0].uiLevelIdc = LEVEL_4_2;
		params.sSpatialLayers[0].sSliceArgument.uiSliceMode = SM_FIXEDSLCNUM_SLICE;
		params.sSpatialLayers[0].sSliceArgument.uiSliceNum = kOpenH264SliceCount;
		params.iMultipleThreadIdc = static_cast<unsigned short>(kOpenH264SliceCount);
		if (encoder->InitializeExt(&params) != 0)
		{
			WelsDestroySVCEncoder(encoder);
			return false;
		}
		int colorFormat = videoFormatI420;
		if (encoder->SetOption(ENCODER_OPTION_DATAFORMAT, &colorFormat) != 0)
		{
			encoder->Uninitialize();
			WelsDestroySVCEncoder(encoder);
			return false;
		}
		m_videoEncoder = encoder;
		m_encoderWidth = width;
		m_encoderHeight = height;
		m_encoderFps = fps;
		m_encoderFrameIndex = 0;
		m_forceNextIdr = true;
		m_lastIdrTimestampUs = 0;
		m_encoderEpochUs = nowUs;
	}

	const auto i420Started = std::chrono::steady_clock::now();
	const size_t ySize = static_cast<size_t>(width) * height;
	m_i420Buffer.resize(ySize * 3 / 2);
	auto& i420 = m_i420Buffer;
	for (uint32_t row = 0; row < height; ++row)
	{
		for (uint32_t column = 0; column < width; ++column)
		{
			const size_t source = (static_cast<size_t>(row) * width + column) * 3;
			const int red = rgb[source];
			const int green = rgb[source + 1];
			const int blue = rgb[source + 2];
			i420[static_cast<size_t>(row) * width + column] = static_cast<uint8_t>(
				((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
		}
	}
	uint8_t* uPlane = i420.data() + ySize;
	uint8_t* vPlane = uPlane + ySize / 4;
	for (uint32_t row = 0; row < height; row += 2)
	{
		for (uint32_t column = 0; column < width; column += 2)
		{
			int sumU = 0;
			int sumV = 0;
			for (uint32_t dy = 0; dy < 2; ++dy)
			{
				for (uint32_t dx = 0; dx < 2; ++dx)
				{
					const size_t source = (static_cast<size_t>(row + dy) * width + column + dx) * 3;
					const int red = rgb[source];
					const int green = rgb[source + 1];
					const int blue = rgb[source + 2];
					sumU += ((-38 * red - 74 * green + 112 * blue + 128) >> 8) + 128;
					sumV += ((112 * red - 94 * green - 18 * blue + 128) >> 8) + 128;
				}
			}
			const size_t chroma = static_cast<size_t>(row / 2) * (width / 2) + column / 2;
			uPlane[chroma] = static_cast<uint8_t>(std::clamp(sumU / 4, 0, 255));
			vPlane[chroma] = static_cast<uint8_t>(std::clamp(sumV / 4, 0, 255));
		}
	}

	RecordVideoMetric(TwinpadVideoMetricStage::I420_CONVERT,
		std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - i420Started).count());

	SSourcePicture picture{};
	picture.iColorFormat = videoFormatI420;
	picture.iPicWidth = width;
	picture.iPicHeight = height;
	picture.iStride[0] = width;
	picture.iStride[1] = width / 2;
	picture.iStride[2] = width / 2;
	picture.pData[0] = i420.data();
	picture.pData[1] = uPlane;
	picture.pData[2] = vPlane;
	picture.uiTimeStamp = static_cast<long long>(m_encoderFrameIndex * 1000 / fps);
	auto* encoder = static_cast<ISVCEncoder*>(m_videoEncoder);
	if ((m_forceNextIdr || nowUs - m_lastIdrTimestampUs >= kIdrIntervalMs * 1000)
		&& encoder->ForceIntraFrame(true) != 0)
		return false;
	SFrameBSInfo info{};
	const auto openH264Started = std::chrono::steady_clock::now();
	const int encodeResult = encoder->EncodeFrame(&picture, &info);
	RecordVideoMetric(TwinpadVideoMetricStage::OPENH264,
		std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - openH264Started).count());
	if (encodeResult != 0 || info.eFrameType == videoFrameTypeSkip || info.eFrameType == videoFrameTypeInvalid)
		return false;

	output.annexB.clear();
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
	output.timestampUs = nowUs >= m_encoderEpochUs ? nowUs - m_encoderEpochUs : 0;
	output.idr = info.eFrameType == videoFrameTypeIDR;
	if (m_forceNextIdr && !output.idr)
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
		return;		m_connectionRequested.store(true);
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
	m_captureGamePad.store(false);		m_connected.store(false);

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
				m_lastFrameQueuedUs.store(0);
				m_receiveBuffer.clear();
				{
					std::scoped_lock lock(m_inputMutex);
					m_inputState = TwinpadInputState{};
					m_gyroMonitorState = TwinpadInputState{};
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
						&& std::string_view(document["type"].GetString()) == "video_config"
						&& document.HasMember("fps") && document["fps"].IsUint())
					{
						SetCaptureFps(static_cast<uint8_t>(std::clamp(document["fps"].GetUint(), 1u, 60u)));
					}
					const auto report = ParseInputReport(document);
					if (report)
					{
						std::scoped_lock lock(m_inputMutex);
						if (!m_hasInputSeq || report->seq > m_inputState.seq)
						{
							m_inputState = *report;
							m_gyroMonitorState = *report;
							m_hasGyroMonitorState = report->gyroValid;
							m_hasInputSeq = true;

							if (report->gyroValid && report->seq % kGyroLogReportInterval == 0)
								cemuLog_log(LogType::Force,
									"[TwinpadGyro] stage=cemu_rx seq={} yaw={} pitch={} roll={}",
									report->seq, report->yaw, report->pitch, report->roll);
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
		"readback_copy", "readback_ready", "pixel_convert", "i420_convert",
		"openh264", "encode_pipeline", "base64_json", "ipc_send",
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

void TwinpadClient::SetCaptureFps(uint8_t fps)
{
	fps = fps >= 1 && fps <= 60 ? fps : 30;
	if (m_captureFps.exchange(fps) == fps)
		return;
	m_lastFrameQueuedUs.store(0);
	if (g_renderer)
		g_renderer->ResetTwinpadCapturePacing();
	ResetVideoEncoder();
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
	constexpr uint64_t kMicrosecondsPerSecond = 1'000'000;
	const uint64_t now = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(
		std::chrono::steady_clock::now().time_since_epoch()).count());
	const uint64_t fps = std::clamp<uint8_t>(m_captureFps.load(), 1, 60);
	const uint64_t interval = kMicrosecondsPerSecond / fps;
	{
		std::scoped_lock lock(m_frameQueueMutex);
		const uint64_t previous = m_lastFrameQueuedUs.load();
		if (now - previous < interval)
			return;
		m_lastFrameQueuedUs.store(now);
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
	while (m_running.load())
	{
		FlushVideoMetricsIfDue();
		std::vector<uint8_t> frameData;
		uint16_t width = 0;
		uint16_t height = 0;
		TwinpadPixelFormat format = TwinpadPixelFormat::Rgb8;
		{
			std::unique_lock lock(m_frameQueueMutex);

			m_frameQueueChanged.wait(lock, [this] { return !m_running.load() || m_hasPendingFrame; });
			if (!m_running.load())
				break;
			if (!m_connected.load())
			{
				m_hasPendingFrame = false;
				continue;
			}
			frameData = std::move(m_pendingFrame);
			width = m_pendingFrameWidth;
			height = m_pendingFrameHeight;
			format = m_pendingFrameFormat;
			m_hasPendingFrame = false;
		}

		const auto pixelConvertStarted = std::chrono::steady_clock::now();
		if (format == TwinpadPixelFormat::Rgb8Srgb)
		{
			std::transform(frameData.begin(), frameData.end(), frameData.begin(), SrgbToRgb);
		}
		else if (format == TwinpadPixelFormat::Rgba8 || format == TwinpadPixelFormat::Rgba8Srgb)
		{
			const size_t pixelCount = static_cast<size_t>(width) * height;
			std::vector<uint8_t> rgb(pixelCount * 3);
			for (size_t pixel = 0; pixel < pixelCount; ++pixel)
			{
				const size_t source = pixel * 4;
				const size_t destination = pixel * 3;
				for (size_t channel = 0; channel < 3; ++channel)
				{
					const uint8_t value = frameData[source + channel];
					rgb[destination + channel] = format == TwinpadPixelFormat::Rgba8Srgb ? SrgbToRgb(value) : value;
				}
			}
			frameData = std::move(rgb);
		}
		RecordVideoMetric(TwinpadVideoMetricStage::PIXEL_CONVERT,
			std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::steady_clock::now() - pixelConvertStarted).count());
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
