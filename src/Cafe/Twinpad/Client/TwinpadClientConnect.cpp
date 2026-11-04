#include "Cafe/Twinpad/Client/TwinpadClient.h"

#include "Common/precompiled.h"

#include <cstring>
#include <string>

#if BOOST_OS_WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
using ClientSocketHandle = SOCKET;
#define TP_SO_ERROR WSAGetLastError()
#define TP_WOULD_BLOCK WSAEWOULDBLOCK
#define TP_INTERRUPTED WSAEINTR
#else
#include <arpa/inet.h>
#include <cerrno>
#include <fcntl.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using ClientSocketHandle = int;
#define TP_SO_ERROR errno
#define TP_WOULD_BLOCK EWOULDBLOCK
#define TP_INTERRUPTED EINTR
#endif

// ---------------------------------------------------------------------------
// TU de ciclo de vida y conectores del cliente Twinpad: Start/Stop,
// SetTitleActive, los tres canales (primario 8421, audio 8422, media 8426),
// los senders compartidos y la lectura del QR (8423).
// ---------------------------------------------------------------------------

namespace
{

void CloseSocketHandle(ClientSocketHandle socket)
{
	if (socket >= 0)
	{
#if BOOST_OS_WINDOWS
		closesocket(socket);
#else
		close(socket);
#endif
	}
}

// Detiene la espera de lectura en curso en el socket (despierta a select/recv
// con un error; verbatim del original).
void ShutdownSocketHandle(ClientSocketHandle socket)
{
#if BOOST_OS_WINDOWS
	::shutdown(socket, SD_BOTH);
#else
	::shutdown(socket, SHUT_RDWR);
#endif
}

// Pone el socket en modo no bloqueante (verbatim del original; el input
// loop de CR-049 marca el ritmo con select en lugar de dormir).
void SetNonBlocking(ClientSocketHandle socket)
{
#if BOOST_OS_WINDOWS
	u_long nonBlocking = 1;
	ioctlsocket(socket, FIONBIO, &nonBlocking);
#else
	const int socketFlags = fcntl(socket, F_GETFL, 0);
	if (socketFlags >= 0)
		fcntl(socket, F_SETFL, socketFlags | O_NONBLOCK);
#endif
}

} // namespace

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
	m_videoSendThread = std::thread(&TwinpadClient::VideoSendLoop, this);
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
	m_videoSendQueueChanged.notify_all();
	{
		std::scoped_lock lock(m_primarySocketMutex);
		if (m_primarySocket >= 0)
			ShutdownSocketHandle(static_cast<ClientSocketHandle>(m_primarySocket));
	}
	{
		std::scoped_lock lock(m_audioSocketMutex);
		if (m_audioSocket >= 0)
			ShutdownSocketHandle(static_cast<ClientSocketHandle>(m_audioSocket));
	}
	if (m_audioThread.joinable())
		m_audioThread.join();
	if (m_frameThread.joinable())
		m_frameThread.join();
	if (m_inputThread.joinable())
		m_inputThread.join();
	if (m_videoSendThread.joinable())
		m_videoSendThread.join();
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
		std::scoped_lock lock(m_videoSendQueueMutex);
		m_videoSendQueue.clear();
	}
	{
		std::scoped_lock primaryLock(m_primarySocketMutex);
		if (m_primarySocket >= 0)
		{
			CloseSocketHandle(static_cast<ClientSocketHandle>(m_primarySocket));
			m_primarySocket = -1;
		}
	}
	{
		std::scoped_lock audioLock(m_audioSocketMutex);
		if (m_audioSocket >= 0)
		{
			SendLine(m_audioSocket, "{\"type\":\"game_audio_stop\",\"game_id\":\"cemu\"}");
			CloseSocketHandle(static_cast<ClientSocketHandle>(m_audioSocket));
			m_audioSocket = -1;
		}
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
				ShutdownSocketHandle(static_cast<ClientSocketHandle>(m_primarySocket));
		}
		{
			std::scoped_lock lock(m_audioSocketMutex);
			if (m_audioSocket >= 0)
				ShutdownSocketHandle(static_cast<ClientSocketHandle>(m_audioSocket));
		}
		{
			// CR-054: idem audio sobre el canal media binario.
			std::scoped_lock lock(m_mediaSocketMutex);
			if (m_mediaSocket >= 0)
				ShutdownSocketHandle(static_cast<ClientSocketHandle>(m_mediaSocket));
		}
		ResetVideoEncoder();
	}
}

bool TwinpadClient::ConnectPrimary()
{
#if BOOST_OS_WINDOWS
	static const bool winsockReady = [] { WSADATA data{}; return WSAStartup(MAKEWORD(2, 2), &data) == 0; }();
	if (!winsockReady)
		return false;
#endif
	std::scoped_lock socketLock(m_primarySocketMutex);
	m_primarySocket = static_cast<std::intptr_t>(socket(AF_INET, SOCK_STREAM, 0));
	if (m_primarySocket < 0)
		return false;
	sockaddr_in endpoint{};
	endpoint.sin_family = AF_INET;
	endpoint.sin_port = htons(twinpad_transport::kGameIpcPort);
	inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
	if (connect(static_cast<ClientSocketHandle>(m_primarySocket), reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0)
	{
		CloseSocketHandle(static_cast<ClientSocketHandle>(m_primarySocket));
		m_primarySocket = -1;
		return false;
	}
	SetNonBlocking(static_cast<ClientSocketHandle>(m_primarySocket));
	const std::string announce = "{\"type\":\"game_announce\",\"game_id\":\"cemu\",\"game_name\":\"Cemu (experimental)\",\"capabilities\":[\"second_screen\",\"audio\"]}";
	if (!SendLine(m_primarySocket, announce))
	{
		CloseSocketHandle(static_cast<ClientSocketHandle>(m_primarySocket));
		m_primarySocket = -1;
		return false;
	}
	// CR-038: si la cámara ya estaba demandada, el canal recién conectado
	// informa de inmediato al Servidor. CR-035: idem micrófono.
	if (m_cameraDemandOn.load())
		SendLine(m_primarySocket, CameraRequestLine(true));
	if (m_micDemandOn.load())
		SendLine(m_primarySocket, MicRequestLine(true));
	// CR-044: idem teclado; el texto base64url vigente viaja con la demanda.
	if (m_keyboardChannel.IsKeyboardDemandOn())
		SendLine(m_primarySocket, KeyboardRequestLine(true, m_keyboardChannel.GetKeyboardDemandTextBase64Url(), m_keyboardChannel.GetKeyboardDemandMaxLength()));
	return true;
}

bool TwinpadClient::ConnectAudio()
{
	std::scoped_lock socketLock(m_audioSocketMutex);
	m_audioSocket = static_cast<std::intptr_t>(socket(AF_INET, SOCK_STREAM, 0));
	if (m_audioSocket < 0)
		return false;
	sockaddr_in endpoint{};
	endpoint.sin_family = AF_INET;
	endpoint.sin_port = htons(twinpad_transport::kAudioIpcPort);
	inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
	if (connect(static_cast<ClientSocketHandle>(m_audioSocket), reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0)
	{
		CloseSocketHandle(static_cast<ClientSocketHandle>(m_audioSocket));
		m_audioSocket = -1;
		return false;
	}
	if (!SendLine(m_audioSocket, "{\"type\":\"game_audio_start\",\"game_id\":\"cemu\",\"rate\":48000,\"channels\":2,\"sample_fmt\":\"s16le\"}"))
	{
		CloseSocketHandle(static_cast<ClientSocketHandle>(m_audioSocket));
		m_audioSocket = -1;
		return false;
	}
	return true;
}

// CR-054: abre el canal media binario (127.0.0.1:8426) y ejecuta el
// handshake NDJSON `game_media_start`. Si el server no tiene listener
// (versión vieja o puerto en uso), falla silenciosamente: el envío de
// video sigue por el NDJSON primario hasta la próxima reconexión.
bool TwinpadClient::ConnectMedia()
{
	std::scoped_lock socketLock(m_mediaSocketMutex);
	if (m_mediaSocket >= 0)
		return true;
	m_mediaSocket = static_cast<std::intptr_t>(socket(AF_INET, SOCK_STREAM, 0));
	if (m_mediaSocket < 0)
		return false;
	sockaddr_in endpoint{};
	endpoint.sin_family = AF_INET;
	endpoint.sin_port = htons(twinpad_transport::kMediaIpcPort);
	inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
	if (connect(static_cast<ClientSocketHandle>(m_mediaSocket), reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0)
	{
		CloseSocketHandle(static_cast<ClientSocketHandle>(m_mediaSocket));
		m_mediaSocket = -1;
		return false;
	}
	// CR-049: el canal media también se abre en no bloqueante (los envíos
	// grandes esperan la ventana de escritura de 2 s de SendBytes).
	SetNonBlocking(static_cast<ClientSocketHandle>(m_mediaSocket));
	if (!SendLine(m_mediaSocket, "{\"type\":\"game_media_start\",\"game_id\":\"cemu\"}"))
	{
		CloseSocketHandle(static_cast<ClientSocketHandle>(m_mediaSocket));
		m_mediaSocket = -1;
		return false;
	}
	m_mediaConnected.store(true);
	cemuLog_log(LogType::Force, "[Twinpad] canal media binario conectado en 127.0.0.1:{} (CR-054).", twinpad_transport::kMediaIpcPort);
	return true;
}

void TwinpadClient::CloseMedia()
{
	std::scoped_lock socketLock(m_mediaSocketMutex);
	m_mediaConnected.store(false);
	if (m_mediaSocket >= 0)
	{
		CloseSocketHandle(static_cast<ClientSocketHandle>(m_mediaSocket));
		m_mediaSocket = -1;
	}
}

// CR-054: header little-endian de 32 bytes + anexo tal cual. Layout:
// magic u16=0x5457 | gfn u64 | ts u64 | idr u8 | w u16 | h u16 | len u32 | fmt u8 | reserved[4].
bool TwinpadClient::SendMediaFrame(const TwinpadQueuedAu& frame)
{
	if (m_mediaSocket < 0 || frame.annexB.empty() || frame.annexB.size() > 0xFFFFFFFFull)
		return false;
	uint8_t header[twinpad_transport::kMediaFrameHeaderBytes]{};
	auto store16 = [&](size_t offset, uint16_t value) {
		header[offset] = static_cast<uint8_t>(value);
		header[offset + 1] = static_cast<uint8_t>(value >> 8);
	};
	auto store32 = [&](size_t offset, uint32_t value) {
		for (size_t byte = 0; byte < 4; ++byte)
			header[offset + byte] = static_cast<uint8_t>(value >> (8 * byte));
	};
	auto store64 = [&](size_t offset, uint64_t value) {
		for (size_t byte = 0; byte < 8; ++byte)
			header[offset + byte] = static_cast<uint8_t>(value >> (8 * byte));
	};
	store16(0, twinpad_transport::kMediaFrameMagic);
	store64(2, frame.gfn);
	store64(10, frame.timestampUs);
	header[18] = frame.idr ? 1 : 0;
	store16(19, frame.width);
	store16(21, frame.height);
	store32(23, static_cast<uint32_t>(frame.annexB.size()));
	header[27] = twinpad_transport::kMediaFrameFmtH264;
	if (!SendBytes(m_mediaSocket, reinterpret_cast<const char*>(header), twinpad_transport::kMediaFrameHeaderBytes))
		return false;
	return SendBytes(m_mediaSocket, reinterpret_cast<const char*>(frame.annexB.data()), frame.annexB.size());
}

bool TwinpadClient::SendLine(std::intptr_t socket, const std::string& line)
{
	return twinpad_transport::SendLine(socket, line);
}

// CR-054: envío crudo de bytes (el canal media no puede llevar el `\n`
// terminal de SendLine: el framing binario no tiene terminador).
bool TwinpadClient::SendBytes(std::intptr_t socket, const char* data, size_t size)
{
	return twinpad_transport::SendBytes(socket, data, size);
}

// CR-023: lee el QR de la app (GET_QR/JSON en 8423); usado por el HUD
// (separa el QR canvas del backend de arriba). Limite de response y
// validacion estricta de filas de 01 son verbatim del original.
bool TwinpadClient::GetCurrentQr(TwinpadQr& qr)
{
#if BOOST_OS_WINDOWS
	static const bool winsockReady = [] { WSADATA data{}; return WSAStartup(MAKEWORD(2, 2), &data) == 0; }();
	if (!winsockReady)
		return false;
#endif
	const ClientSocketHandle socketHandle = socket(AF_INET, SOCK_STREAM, 0);
	if (socketHandle < 0)
		return false;
	sockaddr_in endpoint{};
	endpoint.sin_family = AF_INET;
	endpoint.sin_port = htons(twinpad_transport::kQrIpcPort);
	inet_pton(AF_INET, "127.0.0.1", &endpoint.sin_addr);
	if (connect(socketHandle, reinterpret_cast<sockaddr*>(&endpoint), sizeof(endpoint)) != 0)
	{
		CloseSocketHandle(socketHandle);
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
		CloseSocketHandle(socketHandle);
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
		if (count <= 0)
			break;
		response.append(buffer, static_cast<size_t>(count));
		if (response.size() > twinpad_transport::kMaxQrResponseBytes)
		{
			CloseSocketHandle(socketHandle);
			return false;
		}
		if (response.find('\n') != std::string::npos)
			break;
	}
	CloseSocketHandle(socketHandle);
	const size_t lineEnd = response.find('\n');
	if (lineEnd == std::string::npos)
		return false;
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
	if (rows.Empty() || rows.Size() > 177)
		return false;
	const rapidjson::SizeType width = rows[0].IsString() ? rows[0].GetStringLength() : 0;
	if (width == 0 || width > 177)
		return false;
	parsed.modules.reserve(rows.Size());
	for (const auto& row : rows.GetArray())
	{
		if (!row.IsString() || row.GetStringLength() != width)
			return false;
		std::string modules(row.GetString(), row.GetStringLength());
		if (modules.find_first_not_of("01") != std::string::npos)
			return false;
		parsed.modules.push_back(std::move(modules));
	}
	qr = std::move(parsed);
	return true;
}
