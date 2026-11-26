#include "Cafe/Twinpad/Channels/TwinpadAmiiboChannel.h"

#include "Cafe/Twinpad/Client/TwinpadClient.h"
#include "Cafe/Twinpad/Core/TwinpadTransport.h"
#include "Cafe/OS/libs/nn_nfp/nn_nfp.h"

#include "Common/FileStream.h"
#include "Common/precompiled.h"
#include "config/ActiveSettings.h"

#include <rapidjson/document.h>

#include <algorithm>
#include <atomic>
#include <mutex>
#include <string_view>
#include <vector>

namespace twinpad_amiibo_channel
{
namespace
{

std::atomic<bool> s_amiiboMounted{false};
std::mutex s_amiiboMutex;
uint32 s_lastRefreshMs = 0;

// CR-126: refresco por tick del input loop; el umbral evita tomar el lock de
// `nn::nfp` en cada iteracion del socket.
constexpr uint32 kRefreshIntervalMs = 250;

bool EnsureVirtualTagFile(const fs::path& path, const AmiiboTagPayload& payload)
{
	std::error_code error;
	fs::create_directories(path.parent_path(), error);
	if (error)
		return false;
	// CR-126: si la copia ya existe NO se reimporta el volcado fisico: se
	// conserva el progreso virtual que el juego haya escrito.
	if (fs::exists(path))
		return true;
	FileStream* stream = FileStream::createFile2(path);
	if (!stream)
		return false;
	const bool written = stream->writeData(payload.nfcData.data(), static_cast<sint32>(payload.nfcData.size()))
		== static_cast<sint32>(payload.nfcData.size());
	delete stream;
	return written;
}

} // namespace

bool IsValidUid(std::string_view uid)
{
	if (uid.size() != kUidHexChars)
		return false;
	return std::all_of(uid.begin(), uid.end(), [](char c) {
		return (c >= '0' && c <= '9') || (c >= 'a' && c <= 'f');
	});
}

std::string VirtualTagFileName(std::string_view uid)
{
	return std::string(uid) + ".bin";
}

bool ParseTag(std::string_view line, AmiiboTagPayload& out)
{
	rapidjson::Document document;
	document.Parse(line.data(), line.size());
	if (document.HasParseError() || !document.IsObject())
		return false;
	if (!document.HasMember("type") || !document["type"].IsString()
		|| std::string_view(document["type"].GetString()) != "amiibo_tag")
		return false;
	if (!document.HasMember("present") || !document["present"].IsBool())
		return false;

	// CR-126: `present:false` no lleva `uid` ni `data`; con true lleva ambos.
	const bool present = document["present"].GetBool();
	if (!present)
	{
		if (document.MemberCount() != 2)
			return false;
		out = AmiiboTagPayload{};
		return true;
	}
	if (document.MemberCount() != 4)
		return false;
	if (!document.HasMember("uid") || !document["uid"].IsString()
		|| !document.HasMember("data") || !document["data"].IsString())
		return false;

	std::string uid(document["uid"].GetString(), document["uid"].GetStringLength());
	if (!IsValidUid(uid))
		return false;
	std::vector<uint8_t> dump;
	if (!twinpad_transport::DecodeBase64Url(
			document["data"].GetString(), document["data"].GetStringLength(), dump))
		return false;
	if (dump.size() != kDumpBytes)
		return false;

	out = AmiiboTagPayload{};
	out.present = true;
	out.uid = std::move(uid);
	std::copy_n(dump.begin(), kNfcDataBytes, out.nfcData.begin());
	return true;
}

} // namespace twinpad_amiibo_channel

// CR-126: aplica una lectura NFC validada al HLE `nn::nfp`. La copia virtual
// vive en el userData de Cemu y los cambios del juego van a esa copia.
bool TwinpadClient::FeedAmiiboTag(std::string_view line)
{
	using namespace twinpad_amiibo_channel;

	AmiiboTagPayload payload;
	if (!ParseTag(line, payload))
		return false;

	std::scoped_lock lock(twinpad_amiibo_channel::s_amiiboMutex);
	if (!payload.present)
	{
		if (!twinpad_amiibo_channel::s_amiiboMounted.exchange(false))
			return true;
		nnNfp_unloadAmiibo();
		cemuLog_log(LogType::Force, "[TwinpadAmiibo] tag virtual retirado (copia conservada)");
		return true;
	}

	if (!nnNfp_isInitialized())
	{
		cemuLog_log(LogType::Force, "[TwinpadAmiibo] lectura ignorada: el juego no abrio nn::nfp");
		return false;
	}

	const fs::path directory = ActiveSettings::GetUserDataPath("twinpad/amiibo");
	const fs::path path = directory / VirtualTagFileName(payload.uid);
	if (!EnsureVirtualTagFile(path, payload))
	{
		cemuLog_log(LogType::Force, "[TwinpadAmiibo] no se pudo preparar la copia virtual {}", path.string());
		return false;
	}

	uint32 nfcError = 0;
	if (!nnNfp_touchNfcTagFromFile(path, &nfcError))
	{
		cemuLog_log(LogType::Force, "[TwinpadAmiibo] no se pudo montar el tag virtual (error {})", nfcError);
		twinpad_amiibo_channel::s_amiiboMounted.store(false);
		return false;
	}
	twinpad_amiibo_channel::s_amiiboMounted.store(true);
	twinpad_amiibo_channel::s_lastRefreshMs = GetTickCount();
	cemuLog_log(LogType::Force, "[TwinpadAmiibo] tag virtual montado uid={} copia={}",
		payload.uid, path.string());
	return true;
}

// CR-126: `nnNfp_update` desmonta el tag a los 1500 ms sin re-toque; el
// adaptador refresca la presencia mientras el telefono lo mantiene presente.
void TwinpadClient::RefreshAmiiboPresence()
{
	if (!twinpad_amiibo_channel::s_amiiboMounted.load())
		return;
	if (!m_connected.load())
	{
		ResetAmiibo();
		return;
	}
	const uint32 now = GetTickCount();
	if (now - twinpad_amiibo_channel::s_lastRefreshMs
		< twinpad_amiibo_channel::kRefreshIntervalMs)
		return;
	twinpad_amiibo_channel::s_lastRefreshMs = now;
	nnNfp_refreshAmiiboTouch();
}

void TwinpadClient::ResetAmiibo()
{
	if (!twinpad_amiibo_channel::s_amiiboMounted.exchange(false))
		return;
	nnNfp_unloadAmiibo();
}
