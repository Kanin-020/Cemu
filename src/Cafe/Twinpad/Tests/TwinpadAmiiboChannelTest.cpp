#include "Common/precompiled.h"
#include "Cafe/Twinpad/Channels/TwinpadAmiiboChannel.h"
#include "Cafe/Twinpad/Client/TwinpadClient.h"

#include <cassert>
#include <string>
#include <vector>

// TEST-0223 — verifies SPEC-006 (CR-126): `amiibo_tag` accepts exactly the
// canonical NTAG215 reading (14 lowercase hex UID + 540 B base64url dump),
// rejects incoherent shapes, and projects only the first 532 B into the
// virtual-tag payload consumed by nn::nfp.
void TwinpadAmiiboChannel_test()
{
	using namespace twinpad_amiibo_channel;

	AmiiboTagPayload payload;
	assert(ParseTag(R"({"type":"amiibo_tag","present":false})", payload));
	assert(!payload.present);
	assert(payload.uid.empty());

	std::vector<uint8_t> dump(kDumpBytes, 0x5A);
	const std::string line =
		std::string(R"({"type":"amiibo_tag","present":true,"uid":"0415aabbccddee","data":")")
		+ TwinpadClient::Base64Url(dump.data(), dump.size())
		+ R"("})";
	assert(ParseTag(line, payload));
	assert(payload.present);
	assert(payload.uid == "0415aabbccddee");
	assert(payload.nfcData[0] == 0x5A);
	assert(payload.nfcData[kNfcDataBytes - 1] == 0x5A);

	// `present:false` con datos, volcado corto, UID invalido y campo extra.
	assert(!ParseTag(
		R"({"type":"amiibo_tag","present":false,"uid":"0415aabbccddee","data":"AAEC"})", payload));
	assert(!ParseTag(
		std::string(R"({"type":"amiibo_tag","present":true,"uid":"0415aabbccddee","data":")")
			+ TwinpadClient::Base64Url(dump.data(), dump.size() - 1) + R"("})",
		payload));
	assert(!ParseTag(
		std::string(R"({"type":"amiibo_tag","present":true,"uid":"0415AABBCCDDEE","data":")")
			+ TwinpadClient::Base64Url(dump.data(), dump.size()) + R"("})",
		payload));
	const std::string extra = std::string(line.begin(), line.end() - 1) + R"(,"x":1})";
	assert(!ParseTag(extra, payload));

	assert(VirtualTagFileName("0415aabbccddee") == "0415aabbccddee.bin");
}
