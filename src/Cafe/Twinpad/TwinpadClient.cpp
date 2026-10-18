#include "TwinpadClient.h"

#include <algorithm>
#include <chrono>
#include <thread>
#include <cerrno>
#include <optional>
#include <string_view>
#include <rapidjson/document.h>
#include "Cafe/CafeSystem.h"

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
#include <unistd.h>
#include <fcntl.h>
using SocketLength = socklen_t;
using SocketHandle = int;
static void CloseSocket(SocketHandle socket) { if (socket >= 0) close(socket); }
#endif

namespace
{
constexpr uint16_t kGameIpcPort = 8421;
constexpr uint16_t kAudioIpcPort = 8422;
constexpr uint16_t kQrIpcPort = 8423;
constexpr size_t kMaxQrResponseBytes = 16 * 1024;
constexpr uint64_t kCaptureIntervalMs = 200; // CR-004: 5 fps máximo por readback de screenshot.
constexpr uint64_t kRetryIntervalMs = 2000;
constexpr char kBase64Alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

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
}

bool TwinpadClient::Start(bool captureGamePad)
{
	std::scoped_lock lifecycleLock(m_lifecycleMutex);
	if (m_running.exchange(true))
		return true;
	m_captureGamePad.store(captureGamePad);
	m_lastFrameSentMs.store(0);
	m_lastFrameQueuedMs.store(0);
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
#else
		const ssize_t count = ::send(static_cast<SocketHandle>(socket), payload.data() + sent, payload.size() - sent, MSG_NOSIGNAL);
#endif
		if (count <= 0) return false;
		sent += static_cast<size_t>(count);
	}
	return true;
}

std::string TwinpadClient::Base64Url(const uint8_t* data, size_t size)
{
	std::string result;
	result.reserve((size * 4 + 2) / 3);
	for (size_t i = 0; i < size; i += 3)
	{
		const uint32_t block = (static_cast<uint32_t>(data[i]) << 16)
			| ((i + 1 < size ? static_cast<uint32_t>(data[i + 1]) : 0) << 8)
			| (i + 2 < size ? static_cast<uint32_t>(data[i + 2]) : 0);
		result.push_back(kBase64Alphabet[(block >> 18) & 63]);
		result.push_back(kBase64Alphabet[(block >> 12) & 63]);
		if (i + 1 < size) result.push_back(kBase64Alphabet[(block >> 6) & 63]);
		if (i + 2 < size) result.push_back(kBase64Alphabet[block & 63]);
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
		|| !document["uri"].IsString() || !document.HasMember("modules") || !document["modules"].IsArray())
		return false;
	TwinpadQr parsed;
	parsed.uri = document["uri"].GetString();
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

void TwinpadClient::RequestConnection()
{
	if (!m_titleActive.load() || !CafeSystem::IsTitleRunning())
		return;
	m_connectionRequested.store(true);
	m_captureGamePad.store(true);
	if (!m_running.load())
		Start(true);
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
					m_hasInputSeq = false;
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
			if (connected && m_titleActive.load())
			{
				m_frameNumber.store(0);
				m_audioSequence.store(0);
				m_receiveBuffer.clear();
				{
					std::scoped_lock lock(m_inputMutex);
					m_inputState = TwinpadInputState{};
					m_hasInputSeq = false;
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
			if (m_receiveBuffer.size() > 64 * 1024 && m_receiveBuffer.find('\n') == std::string::npos)
				m_receiveBuffer.clear();
			size_t lineEnd = m_receiveBuffer.find('\n');
			while (lineEnd != std::string::npos)
			{
				rapidjson::Document document;
				document.Parse(m_receiveBuffer.data(), lineEnd);
				if (!document.HasParseError())
				{
					const auto report = ParseInputReport(document);
					if (report)
					{
						std::scoped_lock lock(m_inputMutex);
						if (!m_hasInputSeq || report->seq > m_inputState.seq)
						{
							m_inputState = *report;
							m_hasInputSeq = true;
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
				m_hasInputSeq = false;
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
				m_hasInputSeq = false;
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

void TwinpadClient::PublishGamePadFrame(const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height)
{
	if (!m_connected.load() || !m_captureGamePad.load() || !width || !height
		|| rgb.size() != static_cast<size_t>(width) * height * 3)
		return;
	const uint64_t now = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
	uint64_t previous = m_lastFrameQueuedMs.load();
	if (now - previous < kCaptureIntervalMs || !m_lastFrameQueuedMs.compare_exchange_strong(previous, now))
		return;
	{
		std::scoped_lock lock(m_frameQueueMutex);
		m_pendingFrame = rgb;
		m_pendingFrameWidth = width;
		m_pendingFrameHeight = height;
		m_hasPendingFrame = true;
	}
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
		std::vector<uint8_t> frameData;
		uint16_t width = 0;
		uint16_t height = 0;
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
			frameData = m_pendingFrame;
			width = m_pendingFrameWidth;
			height = m_pendingFrameHeight;
			m_hasPendingFrame = false;
		}

		const uint64_t frame = m_frameNumber.fetch_add(1);
		const std::string line = "{\"type\":\"game_frame\",\"gfn\":" + std::to_string(frame)
			+ ",\"w\":" + std::to_string(width) + ",\"h\":" + std::to_string(height)
			+ ",\"fmt\":\"rgb8\",\"data\":\"" + Base64Url(frameData.data(), frameData.size()) + "\"}";
		std::scoped_lock lock(m_primarySocketMutex);
		if (!m_connected.load() || m_primarySocket < 0)
			continue;
		const uint64_t now = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
		if (!SendLine(m_primarySocket, line))
		{
			m_connected.store(false);
			continue;
		}
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
