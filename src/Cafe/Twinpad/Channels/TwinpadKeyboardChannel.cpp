#include "Cafe/Twinpad/Channels/TwinpadKeyboardChannel.h"

#include "Cafe/Twinpad/Core/TwinpadTransport.h"

#include "Common/precompiled.h"

#include <algorithm>
#include <string_view>

namespace twinpad_keyboard_channel
{
namespace
{

// CR-044: el texto del form del swkbd viaja UTF-8 (convertido de unidades
// UTF-16) y luego en base64url (transplante verbatim del cliente original).
std::string Utf16ToUtf8(const uint16_t* units, size_t count)
{
	std::string out;
	out.reserve(count);
	for (size_t i = 0; i < count; ++i)
	{
		const uint32_t c = units[i];
		if (c < 0x80)
			out.push_back(static_cast<char>(c));
		else if (c < 0x800)
		{
			out.push_back(static_cast<char>(0xC0 | (c >> 6)));
			out.push_back(static_cast<char>(0x80 | (c & 0x3F)));
		}
		else
		{
			out.push_back(static_cast<char>(0xE0 | (c >> 12)));
			out.push_back(static_cast<char>(0x80 | ((c >> 6) & 0x3F)));
			out.push_back(static_cast<char>(0x80 | (c & 0x3F)));
		}
	}
	return out;
}

bool Utf8ToUtf16(std::string_view utf8, std::vector<uint16_t>& out)
{
	out.clear();
	out.reserve(utf8.size());
	for (size_t i = 0; i < utf8.size();)
	{
		const uint8_t b0 = static_cast<uint8_t>(utf8[i]);
		if (b0 < 0x80)
		{
			out.push_back(b0);
			i += 1;
		}
		else if ((b0 & 0xE0) == 0xC0)
		{
			if (i + 1 >= utf8.size())
				return false;
			const uint32_t c = ((b0 & 0x1F) << 6) | (static_cast<uint8_t>(utf8[i + 1]) & 0x3F);
			out.push_back(static_cast<uint16_t>(c));
			i += 2;
		}
		else if ((b0 & 0xF0) == 0xE0)
		{
			if (i + 2 >= utf8.size())
				return false;
			const uint32_t c = ((b0 & 0x0F) << 12) | ((static_cast<uint8_t>(utf8[i + 1]) & 0x3F) << 6)
				| (static_cast<uint8_t>(utf8[i + 2]) & 0x3F);
			out.push_back(static_cast<uint16_t>(c));
			i += 3;
		}
		else
			return false;
	}
	return true;
}

} // namespace

void TwinpadKeyboardChannel::SetDemand(bool on, const uint16_t* text, uint16_t count, uint16_t maxLength)
{
	// CR-044: el ultimo estado se recuerda aunque el IPC no este vivo, para
	// que ConnectPrimary lo reenvie al reconectar (como camara/microfono).
	m_demandOn.store(on);
	if (on)
	{
		m_demandMaxLength.store(std::max<uint16_t>(1, maxLength));
		const std::string utf8 = Utf16ToUtf8(text, text == nullptr ? 0 : count);
		const std::string base64 = twinpad_transport::Base64Url(
			reinterpret_cast<const uint8_t*>(utf8.data()), utf8.size());
		std::scoped_lock lock(m_mutex);
		m_demandTextBase64Url = base64;
	}
}

bool TwinpadKeyboardChannel::IsKeyboardDemandOn() const
{
	return m_demandOn.load(std::memory_order_relaxed);
}

uint16_t TwinpadKeyboardChannel::GetKeyboardDemandMaxLength() const
{
	return m_demandMaxLength.load(std::memory_order_relaxed);
}

std::string TwinpadKeyboardChannel::GetKeyboardDemandTextBase64Url() const
{
	std::scoped_lock lock(m_mutex);
	return m_demandTextBase64Url;
}

bool TwinpadKeyboardChannel::FeedKeyboardInput(std::string_view line)
{
	// CR-044: `keyboard_input` estricto — action conocida y texto base64url
	// UTF-8 que solo `update` lleva (<= 2048 bytes; vacio permitido con
	// `update` para borrar todo el form). Lo incoherente se ignora (E4).
	rapidjson::Document document;
	document.Parse(line.data(), line.size());
	if (document.HasParseError() || !document.IsObject())
		return false;
	if (!document.HasMember("type") || !document["type"].IsString()
		|| std::string_view(document["type"].GetString()) != "keyboard_input")
		return false;
	if (!document.HasMember("action") || !document["action"].IsString())
		return false;
	if (!document.HasMember("text") || !document["text"].IsString())
		return false;
	const std::string_view action = document["action"].GetString();
	TwinpadKeyboardPending::Action pendingAction{};
	if (action == "update")
		pendingAction = TwinpadKeyboardPending::Action::Update;
	else if (action == "confirm")
		pendingAction = TwinpadKeyboardPending::Action::Confirm;
	else if (action == "cancel")
		pendingAction = TwinpadKeyboardPending::Action::Cancel;
	else
		return false;
	const auto text = document["text"].GetString();
	if (pendingAction != TwinpadKeyboardPending::Action::Update && *text != '\0')
		return false;
	TwinpadKeyboardPending pending;
	pending.action = pendingAction;
	if (pendingAction == TwinpadKeyboardPending::Action::Update)
	{
		// El texto completo del form solo viaja con `update` (<= 2048 bytes;
		// vacio es valido: borra todo el texto del form).
		std::vector<uint8_t> decoded;
		if (!twinpad_transport::DecodeBase64Url(text, document["text"].GetStringLength(), decoded))
			return false;
		if (decoded.size() > 2048)
			return false;
		if (!Utf8ToUtf16(std::string_view(reinterpret_cast<const char*>(decoded.data()), decoded.size()), pending.text))
			return false;
	}
	std::scoped_lock lock(m_mutex);
	while (m_pending.size() >= kKeyboardPendingMax)
		m_pending.pop_front();
	m_pending.push_back(std::move(pending));
	return true;
}

bool TwinpadKeyboardChannel::ConsumeKeyboardInput(TwinpadKeyboardPending& out)
{
	std::scoped_lock lock(m_mutex);
	if (m_pending.empty())
		return false;
	out = std::move(m_pending.front());
	m_pending.pop_front();
	return true;
}

} // namespace twinpad_keyboard_channel
