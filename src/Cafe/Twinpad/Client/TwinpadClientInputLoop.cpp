#include "Cafe/Twinpad/Client/TwinpadClient.h"
#include "Cafe/Twinpad/Video/TwinpadCameraDecoder.h"

#include "Common/precompiled.h"

#include <rapidjson/document.h>

#include <chrono>
#include <string>
#include <thread>
#include <optional>

#if BOOST_OS_WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
using InputSocketHandle = SOCKET;
#else
#include <arpa/inet.h>
#include <cerrno>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>
using InputSocketHandle = int;
#endif

// ---------------------------------------------------------------------------
// TU del hilo de input del cliente Twinpad: parseo estricto de input_report,
// loop de reconexion/distribucion (CR-049: select en vez de sleep fijo),
// encolado de motion (CR-045), hooks de conexion del menu de titulos y la
// sonda E2E del hook de pruebas (TWINPAD_E2E_TEST_HOOK).
// ---------------------------------------------------------------------------

#ifdef TWINPAD_E2E_TEST_HOOK
namespace
{
// Test-058 (verbatim del original): la sonda escucha un requisito binario
// del evento de pruebas; la app de test abre el listener de vuelta en
// kE2eCaptureIpcPort inmediatamente.
void StartE2eInputProbe(std::string_view line)
{
	if (line.size() > twinpad_transport::kE2eRequestLimit || !line.starts_with("TEST-058 "))
		return;
	std::string request(line);
	request.push_back('\n');
	std::thread([request = std::move(request)] {
		const InputSocketHandle listener = socket(AF_INET, SOCK_STREAM, 0);
		if (listener < 0)
			return;
		int reuse = 1;
		setsockopt(listener, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));
		sockaddr_in address{};
		address.sin_family = AF_INET;
		address.sin_port = htons(twinpad_transport::kE2eCaptureIpcPort);
		address.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
		if (bind(listener, reinterpret_cast<sockaddr*>(&address), sizeof(address)) != 0 || listen(listener, 1) != 0)
		{
			close(listener);
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
			const InputSocketHandle client = accept(listener, nullptr, nullptr);
			if (client >= 0)
			{
				char input[twinpad_transport::kE2eRequestLimit + 1]{};
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
					const int32_t probeSeq = static_cast<int32_t>(state.seq);
					const std::string probeReport = "{\"type\":\"input_report\",\"seq\":"
						+ std::to_string(probeSeq) + ",\"stick_lx\":0,\"stick_ly\":0,\"stick_rx\":0,\"stick_ry\":0,"
						+ "\"dpad\":8,\"buttons\":0,\"trigger_l2\":0,\"trigger_r2\":0,\"touch_x\":"
						+ std::to_string(state.touchX) + ",\"touch_y\":" + std::to_string(state.touchY)
						+ ",\"touch_pressed\":" + (state.touchPressed ? "true" : "false") + "}";
					if (responseSize > 0)
						::send(client, probeReport.data(), probeReport.size(), 0);
				}
				close(client);
			}
		}
		close(listener);
	}).detach();
}
} // namespace
#endif


namespace
{

// Parseo estricto `input_report` (verbatim del original): rechaza campos
// faltantes o extra, sticks fuera de [-127,127] y motion no finito.
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

} // namespace

void TwinpadClient::RequestConnection()
{
	if (!m_titleActive.load() || !TitleRunning())
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
#if BOOST_OS_WINDOWS
			::shutdown(static_cast<InputSocketHandle>(m_primarySocket), SD_BOTH);
#else
			::shutdown(static_cast<InputSocketHandle>(m_primarySocket), SHUT_RDWR);
#endif
	}
	{
		std::scoped_lock lock(m_audioSocketMutex);
		if (m_audioSocket >= 0)
#if BOOST_OS_WINDOWS
			::shutdown(static_cast<InputSocketHandle>(m_audioSocket), SD_BOTH);
#else
			::shutdown(static_cast<InputSocketHandle>(m_audioSocket), SHUT_RDWR);
#endif
	}
	ResetInputMotionState();
}

void TwinpadClient::InputLoop()
{
	auto nextConnectionAttempt = std::chrono::steady_clock::now();
	bool titleWasActive = false;
	while (m_running.load())
	{
		if (!m_titleActive.load() || !m_connectionRequested.load() || !TitleRunning())
		{
			const bool disconnectRequested = !m_titleActive.load() || !TitleRunning();
			if (titleWasActive || disconnectRequested)
			{
				m_connected.store(false);
				{
					std::scoped_lock lock(m_primarySocketMutex);
					if (m_primarySocket >= 0)
					{
#if BOOST_OS_WINDOWS
						closesocket(static_cast<InputSocketHandle>(m_primarySocket));
#else
						close(static_cast<InputSocketHandle>(m_primarySocket));
#endif
						m_primarySocket = -1;
					}
				}
				{
					std::scoped_lock lock(m_audioSocketMutex);
					if (m_audioSocket >= 0)
					{
#if BOOST_OS_WINDOWS
						closesocket(static_cast<InputSocketHandle>(m_audioSocket));
#else
						close(static_cast<InputSocketHandle>(m_audioSocket));
#endif
						m_audioSocket = -1;
					}
				}
				CloseMedia();
				m_receiveBuffer.clear();
				{
					std::scoped_lock lock(m_audioQueueMutex);
					m_audioQueue.clear();
				}
				// CR-045: el bloque repetido del monolito vive en el helper comun.
				ResetInputMotionState();
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
				{
#if BOOST_OS_WINDOWS
					closesocket(static_cast<InputSocketHandle>(m_primarySocket));
#else
					close(static_cast<InputSocketHandle>(m_primarySocket));
#endif
					m_primarySocket = -1;
				}
			}
			{
				std::scoped_lock lock(m_audioSocketMutex);
				if (m_audioSocket >= 0)
				{
#if BOOST_OS_WINDOWS
					closesocket(static_cast<InputSocketHandle>(m_audioSocket));
#else
					close(static_cast<InputSocketHandle>(m_audioSocket));
#endif
					m_audioSocket = -1;
				}
			}
			CloseMedia();
			const bool connected = ConnectPrimary() && m_titleActive.load() && ConnectAudio();
			// CR-054: el canal media binario se intenta por cada ciclo de
			// reconexión; fallar (server viejo) es barato y el fallback NDJSON
			// lleva el video mientras tanto.
			if (connected)
				ConnectMedia();
			if (connected && m_titleActive.load() && m_connectionRequested.load())
			{
				m_frameNumber.store(0);
				m_audioSequence.store(0);
				ResetVideoEncoder();
				m_receiveBuffer.clear();
				ResetInputMotionState();
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
					{
#if BOOST_OS_WINDOWS
						closesocket(static_cast<InputSocketHandle>(m_primarySocket));
#else
						close(static_cast<InputSocketHandle>(m_primarySocket));
#endif
						m_primarySocket = -1;
					}
				}
				{
					std::scoped_lock lock(m_audioSocketMutex);
					if (m_audioSocket >= 0)
					{
#if BOOST_OS_WINDOWS
						closesocket(static_cast<InputSocketHandle>(m_audioSocket));
#else
						close(static_cast<InputSocketHandle>(m_audioSocket));
#endif
						m_audioSocket = -1;
					}
				}
				CloseMedia();
				nextConnectionAttempt = std::chrono::steady_clock::now()
					+ std::chrono::milliseconds(kRetryIntervalMs);
				continue;
			}
		}
		InputSocketHandle socket = -1;
		{
			std::scoped_lock lock(m_primarySocketMutex);
			socket = static_cast<InputSocketHandle>(m_primarySocket);
		}
		if (socket < 0)
		{
			m_connected.store(false);
			nextConnectionAttempt = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(kRetryIntervalMs);
			continue;
		}
		// CR-049: el socket es NONBLOCK (ConnectPrimary), así que la cadencia de
		// lectura la marca este hilo. Antes era un sleep(10) fijo por iteración:
		// hasta 10 ms de latencia añadida a cada input_report y ráfagas enteras
		// enjauladas en el sleep. Ahora solo espera con select SI no hay línea
		// completa pendiente en el buffer; con datos, drain inmediato.
		if (m_receiveBuffer.find('\n') == std::string::npos)
		{
			fd_set readFds;
			FD_ZERO(&readFds);
			FD_SET(socket, &readFds);
			timeval waitTimeout{};
			waitTimeout.tv_sec = 0;
			waitTimeout.tv_usec = 2000; // 2 ms: reacción de reconexión intacta
#if BOOST_OS_WINDOWS
			const int ready = ::select(0, &readFds, nullptr, nullptr, &waitTimeout);
#else
			const int ready = ::select(socket + 1, &readFds, nullptr, nullptr, &waitTimeout);
#endif
			if (ready <= 0)
				continue;
		}
		char buffer[8192];
#if BOOST_OS_WINDOWS
		const int count = ::recv(socket, buffer, sizeof(buffer) - 1, 0);
#else
		const ssize_t count = ::recv(socket, buffer, sizeof(buffer) - 1, MSG_DONTWAIT);
#endif
		if (count > 0)
		{
			m_receiveBuffer.append(buffer, static_cast<size_t>(count));
#ifdef TWINPAD_E2E_TEST_HOOK
			const auto probeEnd = m_receiveBuffer.find("TEST-058 ");
			if (probeEnd != std::string::npos)
			{
				const auto probeLineEnd = m_receiveBuffer.find('\n', probeEnd);
				if (probeLineEnd != std::string::npos)
				{
					StartE2eInputProbe(std::string_view(m_receiveBuffer).substr(probeEnd, probeLineEnd - probeEnd));
					m_receiveBuffer.erase(probeEnd, probeLineEnd - probeEnd + 1);
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
						&& std::string_view(document["type"].GetString()) == "video_force_idr")
					{
						// CR-051: el teléfono entró al canal media; IDR inmediata en el
						// siguiente frame publicado en lugar de la cadencia periódica.
						RequestKeyframe();
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
						// drop-old que consume el HLE `mic` bajo demanda (canal twinpad).
						FeedMicChunk(std::string_view(m_receiveBuffer.data(), lineEnd));
					}
					else if (document.IsObject() && document.HasMember("type") && document["type"].IsString()
						&& std::string_view(document["type"].GetString()) == "keyboard_input")
					{
						// CR-044: texto/acción del IME del teléfono; se valida estrictamente
						// y queda pendiente para el hilo de render del HLE `swkbd`.
						FeedKeyboardInput(std::string_view(m_receiveBuffer.data(), lineEnd));
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
							// CR-045: cada muestra con motion valido se encola para el
							// drenaje por frame del VPAD (sin drop de muestras intermedias).
							if (report->motionValid)
							{
								TwinpadMotionSample pending{};
								pending.timestampUs = report->motionTimestampUs;
								pending.seq = report->seq;
								pending.accel = report->accel;
								pending.gyro = report->gyro;
								m_pendingMotionSamples.push_back(std::move(pending));
								if (m_pendingMotionSamples.size() > kMotionQueueMaxSamples)
									m_pendingMotionSamples.erase(m_pendingMotionSamples.begin());
							}

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
				{
#if BOOST_OS_WINDOWS
					closesocket(static_cast<InputSocketHandle>(m_primarySocket));
#else
					close(static_cast<InputSocketHandle>(m_primarySocket));
#endif
					m_primarySocket = -1;
				}
			}
			{
				std::scoped_lock lock(m_audioSocketMutex);
				if (m_audioSocket >= 0)
				{
#if BOOST_OS_WINDOWS
					closesocket(static_cast<InputSocketHandle>(m_audioSocket));
#else
					close(static_cast<InputSocketHandle>(m_audioSocket));
#endif
					m_audioSocket = -1;
				}
			}
			CloseMedia();
			ResetInputMotionState();
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
				{
#if BOOST_OS_WINDOWS
					closesocket(static_cast<InputSocketHandle>(m_primarySocket));
#else
					close(static_cast<InputSocketHandle>(m_primarySocket));
#endif
					m_primarySocket = -1;
				}
			}
			{
				std::scoped_lock lock(m_audioSocketMutex);
				if (m_audioSocket >= 0)
				{
#if BOOST_OS_WINDOWS
					closesocket(static_cast<InputSocketHandle>(m_audioSocket));
#else
					close(static_cast<InputSocketHandle>(m_audioSocket));
#endif
					m_audioSocket = -1;
				}
			}
			CloseMedia();
			ResetInputMotionState();
			{
				std::scoped_lock lock(m_audioQueueMutex);
				m_audioQueue.clear();
			}
			nextConnectionAttempt = std::chrono::steady_clock::now()
				+ std::chrono::milliseconds(kRetryIntervalMs);
		}
		// CR-049: sin sleep fijo al final del loop — el select de arriba es el
		// único punto de espera (≤2 ms sin datos; 0 con datos recientes).
	}
}
