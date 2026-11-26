#pragma once

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <string_view>

// Canal Amiibo del modulo Twinpad (CR-126): valida `amiibo_tag` y proyecta la
// lectura NFC del telefono sobre el tag virtual del HLE `nn::nfp`. El telefono
// entrega el volcado NTAG215 completo (540 B); Cemu consume los primeros 532 B
// (`AmiiboRawNFCData`) y persiste una copia por UID.

namespace twinpad_amiibo_channel
{

// CR-126: UID NTAG215 de 7 bytes en hex minuscula.
constexpr size_t kUidHexChars = 14;

// CR-126: volcado NTAG215 completo (135 paginas x 4 B).
constexpr size_t kDumpBytes = 540;

// CR-126: memoria NTAG215 que consume Cemu (`sizeof(AmiiboRawNFCData)`).
constexpr size_t kNfcDataBytes = 532;

struct AmiiboTagPayload
{
	bool present = false;
	std::string uid;
	std::array<uint8_t, kNfcDataBytes> nfcData{};
};

// CR-126: `amiibo_tag` estricto (campos exactos, UID hex de 14, volcado de 540
// B). Lo incoherente se ignora sin montar ni desmontar el tag vigente.
bool ParseTag(std::string_view line, AmiiboTagPayload& out);

bool IsValidUid(std::string_view uid);

//! Nombre de la copia virtual: `<uid>.bin` (UID ya validado).
std::string VirtualTagFileName(std::string_view uid);

} // namespace twinpad_amiibo_channel
