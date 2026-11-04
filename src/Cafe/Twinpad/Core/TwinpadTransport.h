#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

// Capa de transporte del modulo Twinpad: sockets TCP loopback, envio de
// lineas/bytes NDJSON, codificacion base64url compartida (demands de
// camera/mic/keyboard) y constantes de puertos/framing media (CR-054).
// Transplante verbatim del cliente original; el cliente solo delega aqui
// (SRP: nada de estado del cliente en este modulo). Los detalles winsock
// estan confinados al .cpp: la API publica usa std::intptr_t para el socket.

namespace twinpad_transport
{

// Puertos del servidor local (game/audio/QR/E2E/media CR-054).
constexpr uint16_t kGameIpcPort = 8421;
constexpr uint16_t kAudioIpcPort = 8422;
constexpr uint16_t kMediaIpcPort = 8426; // CR-054: canal media binario
constexpr uint16_t kQrIpcPort = 8423;
#ifdef TWINPAD_E2E_TEST_HOOK
constexpr uint16_t kE2eCaptureIpcPort = 8424;
constexpr size_t kE2eRequestLimit = 128;
#endif
constexpr size_t kMaxQrResponseBytes = 16 * 1024;
// CR-054: framing del canal media binario (header LE de 32 bytes).
constexpr size_t kMediaFrameHeaderBytes = 32;
constexpr uint16_t kMediaFrameMagic = 0x5457; // "TW"
constexpr uint8_t kMediaFrameFmtH264 = 1;

// Escritura no bloqueante con select() de 2 s; EINTR/WSAEINTR reintenta
// en bucle (verbatim del original).
bool WaitForSocketWritable(std::intptr_t socketHandle);

// CR-054: envio de una linea NDJSON (anade el '\n' terminal) y de bytes
// crudos para el canal media (sin terminador: framing binario).
bool SendLine(std::intptr_t socketHandle, const std::string& line);
bool SendBytes(std::intptr_t socketHandle, const char* data, size_t size);

// base64url sin padding compartido por camera/mic demands y el teclado del
// form (CR-044).
std::string Base64Url(const uint8_t* data, size_t size);
// CR-035/CR-044: decodificacion estricta de base64url sin relleno; rechaza
// la cadena entera ante cualquier caracter invalido o tamano 4n+1. Vive en
// transport porque mic_chunk y keyboard_input comparten la misma semantica.
bool DecodeBase64Url(const char* text, size_t size, std::vector<uint8_t>& out);

} // namespace twinpad_transport
