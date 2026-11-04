#include "Cafe/Twinpad/Core/TwinpadTransport.h"

#include "Common/precompiled.h"

#include <cerrno>
#include <cstring>

#if BOOST_OS_WINDOWS
#include <winsock2.h>
#include <ws2tcpip.h>
using SocketLength = int;
using SocketHandle = SOCKET;
static void CloseSocket(SocketHandle socket) { if (socket >= 0) closesocket(socket); }
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>
using SocketLength = socklen_t;
using SocketHandle = int;
static void CloseSocket(SocketHandle socket) { if (socket >= 0) close(socket); }
#endif

namespace twinpad_transport
{

namespace
{

constexpr char kBase64UrlAlphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";

bool WaitForSocketWritableImpl(SocketHandle socket)
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

// Ruta interna de SendBytes: el while envia bytes por trozos hasta cubrir
// el size pedido; a bloqueo se espera la ventana de escritura de 2 s antes
// de reintentar (verbatim del original).
bool SendBytesImpl(SocketHandle socket, const char* data, size_t size)
{
	size_t sent = 0;
	while (sent < size)
	{
#if BOOST_OS_WINDOWS
		const int count = ::send(socket, data + sent, static_cast<int>(size - sent), 0);
		if (count == SOCKET_ERROR)
		{
			const int error = WSAGetLastError();
			if (error == WSAEINTR)
				continue;
			if (error != WSAEWOULDBLOCK || !WaitForSocketWritableImpl(socket))
				return false;
			continue;
		}
#else
		const ssize_t count = ::send(socket, data + sent, size - sent, MSG_NOSIGNAL);
		if (count < 0)
		{
			if (errno == EINTR)
				continue;
			if ((errno != EAGAIN && errno != EWOULDBLOCK)
				|| !WaitForSocketWritableImpl(socket))
				return false;
			continue;
		}
#endif
		if (count == 0) return false;
		sent += static_cast<size_t>(count);
	}
	return true;
}

} // namespace

bool WaitForSocketWritable(std::intptr_t socketHandle)
{
	return WaitForSocketWritableImpl(static_cast<SocketHandle>(socketHandle));
}

bool SendLine(std::intptr_t socketHandle, const std::string& line)
{
	if (socketHandle < 0)
		return false;
	const std::string payload = line + "\n";
	return SendBytesImpl(static_cast<SocketHandle>(socketHandle), payload.data(), payload.size());
}

// CR-054: envio crudo de bytes (el canal media no puede llevar el '\n'
// terminal de SendLine: el framing binario no tiene terminador).
bool SendBytes(std::intptr_t socketHandle, const char* data, size_t size)
{
	if (socketHandle < 0)
		return false;
	return SendBytesImpl(static_cast<SocketHandle>(socketHandle), data, size);
}

std::string Base64Url(const uint8_t* data, size_t size)
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
		output[outputIndex++] = kBase64UrlAlphabet[(block >> 18) & 63];
		output[outputIndex++] = kBase64UrlAlphabet[(block >> 12) & 63];
		output[outputIndex++] = kBase64UrlAlphabet[(block >> 6) & 63];
		output[outputIndex++] = kBase64UrlAlphabet[block & 63];
		inputIndex += 3;
	}
	if (remainder != 0)
	{
		const uint32_t block = (static_cast<uint32_t>(data[inputIndex]) << 16)
			| (remainder == 2 ? static_cast<uint32_t>(data[inputIndex + 1]) << 8 : 0);
		output[outputIndex++] = kBase64UrlAlphabet[(block >> 18) & 63];
		output[outputIndex++] = kBase64UrlAlphabet[(block >> 12) & 63];
		if (remainder == 2)
			output[outputIndex] = kBase64UrlAlphabet[(block >> 6) & 63];
	}
	return result;
}

// CR-035: decodificador base64url sin relleno (mismo contrato que core;
// duplicado de TwinpadCameraDecoder porque el codec H.264 no aplica aqui).
// RECHAZA la cadena entera ante cualquier caracter invalido o tamano 4n+1
// (estricto del CR-035/CR-044: mic_chunk y keyboard_input nunca adivinan).
namespace
{

int DecodeBase64UrlCharBase(char c)
{
	if (c >= 'A' && c <= 'Z') return c - 'A';
	if (c >= 'a' && c <= 'z') return c - 'a' + 26;
	if (c >= '0' && c <= '9') return c - '0' + 52;
	if (c == '-') return 62;
	if (c == '_') return 63;
	return -1;
}

} // namespace

bool DecodeBase64Url(const char* text, size_t size, std::vector<uint8_t>& out)
{
	int value = 0;
	out.clear();
	if (size == 0 || size % 4 == 1)
		return false;
	uint32_t buffer = 0;
	int bits = 0;
	for (size_t i = 0; i < size; ++i)
	{
		value = DecodeBase64UrlCharBase(text[i]);
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

} // namespace twinpad_transport
