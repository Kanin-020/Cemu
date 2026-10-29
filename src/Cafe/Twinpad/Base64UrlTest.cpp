#include "Common/precompiled.h"
#include "Cafe/Twinpad/TwinpadClient.h"
#include "Cafe/Twinpad/TwinpadCameraDecoder.h"
#include "Cafe/HW/Latte/Renderer/Vulkan/VulkanRenderer.h"

#include <initializer_list>

namespace
{
std::string Base64UrlReference(const uint8_t* data, size_t size)
{
	static constexpr char alphabet[] = "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_";
	std::string encoded;
	for (size_t index = 0; index < size; index += 3)
	{
		const uint32_t block = (static_cast<uint32_t>(data[index]) << 16)
			| (index + 1 < size ? static_cast<uint32_t>(data[index + 1]) << 8 : 0)
			| (index + 2 < size ? static_cast<uint32_t>(data[index + 2]) : 0);
		encoded.push_back(alphabet[(block >> 18) & 63]);
		encoded.push_back(alphabet[(block >> 12) & 63]);
		if (index + 1 < size)
			encoded.push_back(alphabet[(block >> 6) & 63]);
		if (index + 2 < size)
			encoded.push_back(alphabet[block & 63]);
	}
	return encoded;
}
}

void VulkanRenderer_twinpad_readback_test()
{
	// TEST-075 — verifies SPEC-003: completed Twinpad readbacks are chosen newest-first without waiting.
	using Slot = VulkanRenderer::TwinpadReadbackSlot;
	std::array<Slot, VulkanRenderer::TWINPAD_READBACK_SLOT_COUNT> slots{};
	slots[0].pending = true;
	slots[0].sequence = 8;
	slots[0].commandBufferId = 10;
	slots[1].pending = true;
	slots[1].sequence = 9;
	slots[1].commandBufferId = 12;
	slots[2].pending = true;
	slots[2].sequence = 7;
	slots[2].commandBufferId = 9;
	cemu_assert_debug(VulkanRenderer::FindNewestCompletedTwinpadReadbackSlot(slots, 11) == 0);
	cemu_assert_debug(VulkanRenderer::FindNewestCompletedTwinpadReadbackSlot(slots, 13) == 1);
	cemu_assert_debug(VulkanRenderer::FindNewestCompletedTwinpadReadbackSlot(slots, 9) == slots.size());
	slots[1].pending = false;
	slots[2].pending = false;
	cemu_assert_debug(VulkanRenderer::FindNewestCompletedTwinpadReadbackSlot(slots, 13) == 0);

	// A partially allocated ring may have no completed slot while every
	// allocated slot is pending; callers must compare against slots.size().
	slots[0].pending = true;
	slots[0].commandBufferId = 13;
	slots[1].pending = true;
	slots[1].commandBufferId = 14;
	const size_t noCompletedSlot = VulkanRenderer::FindOldestCompletedTwinpadReadbackSlot(slots, 13);
	cemu_assert_debug(noCompletedSlot == slots.size());
	cemu_assert_debug(noCompletedSlot != 2);
}



void TwinpadClient_base64url_test()
{
	const auto encode = [](std::initializer_list<uint8_t> bytes) {
		return TwinpadClient::Base64Url(bytes.begin(), bytes.size());
	};
	cemu_assert_debug(encode({}) == "");
	cemu_assert_debug(encode({0x66}) == "Zg");
	cemu_assert_debug(encode({0x66, 0x6f}) == "Zm8");
	cemu_assert_debug(encode({0x66, 0x6f, 0x6f}) == "Zm9v");
	cemu_assert_debug(encode({0xfb, 0xff}) == "-_8");

	std::vector<uint8_t> frame(1280 * 720 * 3);
	for (size_t index = 0; index < frame.size(); ++index)
		frame[index] = static_cast<uint8_t>((index * 37 + index / 19) % 256);
	const auto encoded = TwinpadClient::Base64Url(frame.data(), frame.size());
	cemu_assert_debug(encoded == Base64UrlReference(frame.data(), frame.size()));
	cemu_assert_debug(encoded.size() == (frame.size() * 4 + 2) / 3);
	cemu_assert_debug(encoded.find_first_not_of("ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789-_") == std::string::npos);
}

namespace
{
struct H264ParameterSets
{
	bool sps{};
	bool pps{};
	bool idr{};
	bool contractCompliant{};
};

H264ParameterSets InspectAnnexB(const std::vector<uint8_t>& bytes)
{
	H264ParameterSets result;
	auto findStartCode = [&bytes](size_t from, size_t& prefixSize) {
		for (size_t index = from; index + 3 <= bytes.size(); ++index)
		{
			if (bytes[index] != 0 || bytes[index + 1] != 0)
				continue;
			if (bytes[index + 2] == 1)
			{
				prefixSize = 3;
				return index;
			}
			if (index + 4 <= bytes.size() && bytes[index + 2] == 0 && bytes[index + 3] == 1)
			{
				prefixSize = 4;
				return index;
			}
		}
		return bytes.size();
	};

	size_t prefixSize = 0;
	size_t start = findStartCode(0, prefixSize);
	while (start < bytes.size())
	{
		const size_t nal = start + prefixSize;
		size_t nextPrefixSize = 0;
		const size_t next = findStartCode(nal + 1, nextPrefixSize);
		if (nal < bytes.size())
		{
			const uint8_t type = bytes[nal] & 0x1f;
			result.sps |= type == 7;
			result.pps |= type == 8;
			result.idr |= type == 5;
			if (type == 7 && nal + 3 < bytes.size())
			{
				const uint8_t profileIdc = bytes[nal + 1];
				const uint8_t constraints = bytes[nal + 2];
				const uint8_t levelIdc = bytes[nal + 3];
				// CR-041: contrato del stream relajado — Baseline (66) o Main (77), nivel ≤4.2.
				result.contractCompliant |= (profileIdc == 66 || profileIdc == 77) && levelIdc != 0 && levelIdc <= 42;
			}
		}
		start = next;
		prefixSize = nextPrefixSize;
	}
	return result;
}
}

void TwinpadClient_h264_encoder_test()
{
	// TEST-073 — verifies SPEC-003/006: real OpenH264 access units carry Annex B SPS/PPS and IDR metadata.
	auto& client = TwinpadClient::Instance();
	client.SetHardwareAccelerationEnabled(false);
	client.ResetVideoEncoder();
	cemu_assert_debug(!client.IsHardwareAccelerationEnabled());
	std::vector<uint8_t> frame(64 * 64 * 3);
	for (size_t index = 0; index < frame.size(); ++index)
		frame[index] = static_cast<uint8_t>((index * 37 + index / 19) % 256);

	TwinpadEncodedFrame first;
	const bool firstEncoded = client.EncodeGamePadFrame(frame, 64, 64, first);
	cemu_assert_debug(firstEncoded);
	if (firstEncoded)
	{
		const auto nals = InspectAnnexB(first.annexB);
		cemu_assert_debug(first.idr);
		cemu_assert_debug(nals.idr);
		cemu_assert_debug(nals.sps);
		cemu_assert_debug(nals.pps);
		cemu_assert_debug(nals.contractCompliant);
	}
	cemu_assert_debug(!client.IsHardwareEncoderActive());
	std::this_thread::sleep_for(std::chrono::milliseconds(1));
	TwinpadEncodedFrame second;
	const bool secondEncoded = client.EncodeGamePadFrame(frame, 64, 64, second);
	cemu_assert_debug(secondEncoded);
	if (secondEncoded)
		cemu_assert_debug(second.timestampUs > first.timestampUs);

	client.ResetVideoEncoder();
	TwinpadEncodedFrame afterReset;
	const bool resetEncoded = client.EncodeGamePadFrame(frame, 64, 64, afterReset);
	cemu_assert_debug(resetEncoded);
	if (resetEncoded)
	{
		const auto nals = InspectAnnexB(afterReset.annexB);
		cemu_assert_debug(afterReset.idr);
		cemu_assert_debug(nals.idr);
		cemu_assert_debug(nals.sps);
		cemu_assert_debug(nals.pps);
		cemu_assert_debug(nals.contractCompliant);
	}

	client.ResetVideoEncoder();
	client.SetHardwareAccelerationEnabled(false);
}

void TwinpadClient_hardware_encoder_test()
{
	// TEST-082 — verifies CR-020: hardware encoding is opt-in and uses the same H.264 contract when available.
	auto& client = TwinpadClient::Instance();
	client.SetHardwareAccelerationEnabled(false);
	client.ResetVideoEncoder();
	cemu_assert_debug(!client.IsHardwareAccelerationEnabled());

	std::vector<uint8_t> frame(64 * 64 * 3, 128);
	TwinpadEncodedFrame software;
	const bool softwareEncoded = client.EncodeGamePadFrame(frame, 64, 64, software);
	cemu_assert_debug(softwareEncoded);
	if (!softwareEncoded)
	{
		client.ResetVideoEncoder();
		return;
	}
	cemu_assert_debug(!client.IsHardwareEncoderActive());

	client.SetHardwareAccelerationEnabled(true);
	cemu_assert_debug(client.IsHardwareAccelerationEnabled());
	TwinpadEncodedFrame accelerated;
	const bool acceleratedEncoded = client.EncodeGamePadFrame(frame, 64, 64, accelerated);
	cemu_assert_debug(acceleratedEncoded);
	if (acceleratedEncoded)
	{
		const auto nals = InspectAnnexB(accelerated.annexB);
		cemu_assert_debug(accelerated.idr);
		cemu_assert_debug(nals.idr);
		cemu_assert_debug(nals.sps);
		cemu_assert_debug(nals.pps);
		cemu_assert_debug(nals.contractCompliant);
	}

	client.SetHardwareAccelerationEnabled(false);
	TwinpadEncodedFrame disabled;
	const bool disabledEncoded = client.EncodeGamePadFrame(frame, 64, 64, disabled);
	cemu_assert_debug(disabledEncoded);
	cemu_assert_debug(!client.IsHardwareEncoderActive());
	client.ResetVideoEncoder();
}

// TEST-096 — verifies SPEC-006 (CR-034): camera_frame is accepted only as 320x240 with
// valid base64url H.264 data, the decoder reports no frame until one decodes, and a
// decoded 4:2:0 frame lands in the CAM surface as NV12 (full Y plane + interleaved UV).
void TwinpadCameraDecoder_camera_frame_test()
{
	auto& decoder = TwinpadCameraDecoder::Instance();
	cemu_assert_debug(!decoder.HasFrame());
	cemu_assert_debug(!decoder.DecodeLine(R"({"type":"camera_frame","w":640,"h":480,"ts_capture_us":0,"idr":true,"data":"AAEC"})"));
	cemu_assert_debug(!decoder.DecodeLine(R"({"type":"camera_frame","w":320,"h":240,"ts_capture_us":0,"idr":true,"data":"!!"})"));
	cemu_assert_debug(!decoder.DecodeLine(R"({"type":"camera_frame","w":320,"h":240,"ts_capture_us":0,"idr":true,"data":"AAEC"})"));
	cemu_assert_debug(!decoder.DecodeLine(R"({"type":"rumble"})"));
	cemu_assert_debug(!decoder.HasFrame());
	uint8_t out[16];
	cemu_assert_debug(decoder.CopyLatestNv12(out, sizeof(out)) == 0);

	// 4x2 con stride mayor que el ancho (alineación 16) para probar el muestreo.
	uint8_t luma[2 * 16];
	uint8_t chromaU[1 * 16];
	uint8_t chromaV[1 * 16];
	for (int row = 0; row < 2; ++row)
		for (int column = 0; column < 16; ++column)
			luma[row * 16 + column] = static_cast<uint8_t>(row * 16 + column);
	for (int column = 0; column < 16; ++column)
	{
		chromaU[column] = static_cast<uint8_t>(100 + column);
		chromaV[column] = static_cast<uint8_t>(200 + column);
	}

	std::vector<uint8_t> nv12;
	cemu_assert_debug(TwinpadCameraDecoder::ConvertToNv12(
		luma, 16, chromaU, chromaV, 16, 4, 2, nv12));
	cemu_assert_debug(nv12.size() == 4 * 2 + 4 * 2 / 2);
	cemu_assert_debug(nv12.size() == 12);
	if (nv12.size() == 12)
	{
		// Y completo, fila a fila, sin el relleno del stride.
		cemu_assert_debug(nv12[0] == 0 && nv12[3] == 3);
		cemu_assert_debug(nv12[4] == 16 && nv12[7] == 19);
		// UV intercalado: dos pares por fila de croma.
		cemu_assert_debug(nv12[8] == 100 && nv12[9] == 200);
		cemu_assert_debug(nv12[10] == 101 && nv12[11] == 201);
	}

	// Geometría impar o puntero nulo: no produce buffer.
	std::vector<uint8_t> rejected;
	cemu_assert_debug(!TwinpadCameraDecoder::ConvertToNv12(
		luma, 16, chromaU, chromaV, 16, 3, 2, rejected));
	cemu_assert_debug(!TwinpadCameraDecoder::ConvertToNv12(
		nullptr, 16, chromaU, chromaV, 16, 4, 2, rejected));
	cemu_assert_debug(rejected.empty());
}// TEST-0100 — verifies SPEC-006 (CR-038): the camera demand has exactly two canonical
// wire lines and is remembered safely while the IPC channel is offline (it will be
// re-sent by ConnectPrimary after game_announce).
void TwinpadClient_camera_request_test()
{
	cemu_assert_debug(TwinpadClient::CameraRequestLine(true)
		== R"({"type":"camera_request","on":true})");
	cemu_assert_debug(TwinpadClient::CameraRequestLine(false)
		== R"({"type":"camera_request","on":false})");

	// Sin conexión, recordar la demanda no debe crashear ni corromper estado.
	TwinpadClient::Instance().PublishCameraRequest(true);
	cemu_assert_debug(TwinpadClient::Instance().m_cameraDemandOn.load() == true);
	TwinpadClient::Instance().PublishCameraRequest(false);
	cemu_assert_debug(TwinpadClient::Instance().m_cameraDemandOn.load() == false);
}

// TEST-0103 — verifies SPEC-006 (CR-035): the microphone demand has exactly two
// canonical wire lines, wire `mic_chunk` is strictly validated (sample/byte
// coherence), it enqueues FIFO as host-order s16 and a warm queue feeds
// ConsumeMicSamples only under fresh demand.
void TwinpadClient_mic_test()
{
	cemu_assert_debug(TwinpadClient::MicRequestLine(true)
		== R"({"type":"mic_request","on":true})");
	cemu_assert_debug(TwinpadClient::MicRequestLine(false)
		== R"({"type":"mic_request","on":false})");

	// Sin conexión, recordar la demanda no debe crashear ni corromper estado.
	TwinpadClient::Instance().PublishMicRequest(true);
	cemu_assert_debug(TwinpadClient::Instance().m_micDemandOn.load() == true);
	TwinpadClient::Instance().PublishMicRequest(false);
	cemu_assert_debug(TwinpadClient::Instance().m_micDemandOn.load() == false);

	auto& client = TwinpadClient::Instance();
	// Chunk canónico: 32 muestras (1 ms @ 32 kHz); muestras 0x0001 y 0xFFFF en s16le.
	std::vector<uint8_t> pcm(64, 0);
	pcm[0] = 0x01; pcm[1] = 0x00; pcm[2] = 0xFF; pcm[3] = 0xFF;
	const std::string encoded = TwinpadClient::Base64Url(pcm.data(), pcm.size());
	const std::string chunkLine
		= "{\"type\":\"mic_chunk\",\"samples\":32,\"data\":\"" + encoded + "\"}";
	cemu_assert_debug(client.FeedMicChunk(chunkLine));

	// Coherencia: bytes que no corresponden a `samples` o muestras fuera
	// de rango se ignoran (E4 del CR-035).
	const std::string badCount = "{\"type\":\"mic_chunk\",\"samples\":33,\"data\":\"" + encoded + "\"}";
	cemu_assert_debug(!client.FeedMicChunk(badCount));
	const std::string badType = "{\"type\":\"mic_chunk\",\"samples\":32,\"data\":\"!!\"}";
	cemu_assert_debug(!client.FeedMicChunk(badType));
	const std::string badMin = "{\"type\":\"mic_chunk\",\"samples\":31,\"data\":\"AAAA\"}";
	cemu_assert_debug(!client.FeedMicChunk(badMin));

	// Demanda vigente + cola fresca → s16 host-order FIFO y consumo completo.
	client.PublishMicRequest(true);
	int16_t sink[32] = {};
	cemu_assert_debug(client.ConsumeMicSamples(sink, 32));
	cemu_assert_debug(sink[0] == 1 && sink[1] == -1 && sink[2] == 0);
	// La cola ya está vacía: sin datos suficientes no se consumen parciales.
	cemu_assert_debug(!client.ConsumeMicSamples(sink, 32));
	client.PublishMicRequest(false);
}


void TwinpadClient_drc_scale_test()
{
	// TEST-088 — verifies SPEC-003/CR-030: the GamePad (DRC) emission resolution
	// is applied by nearest-neighbor scaling before the encoder, without upscale.
	auto& client = TwinpadClient::Instance();
	std::vector<uint8_t> source(2 * 2 * 3);
	// Cuadrantes: izquierda azul, derecha roja (verificables por canal).
	source[0] = 0; source[1] = 0; source[2] = 255; // píxel (0,0)
	source[3] = 255; source[4] = 0; source[5] = 0; // píxel (1,0)
	source[6] = 0; source[7] = 0; source[8] = 255; // píxel (0,1)
	source[9] = 255; source[10] = 0; source[11] = 0; // píxel (1,1)

	// Escala 2×2 → 2×2: identidad (los targets >= nativos no escalan).
	const auto identity = TwinpadClient::ScaleRgb(source, 2, 2, 2, 2);
	cemu_assert_debug(identity.size() == source.size());
	cemu_assert_debug(identity == source);

	// Escala 2×2 → 1×1: nearest muestrea el píxel superior izquierdo.
	const auto shrunk = TwinpadClient::ScaleRgb(source, 2, 2, 1, 1);
	cemu_assert_debug(shrunk.size() == 3);
	if (shrunk.size() == 3)
	{
		cemu_assert_debug(shrunk[0] == 0);
		cemu_assert_debug(shrunk[1] == 0);
		cemu_assert_debug(shrunk[2] == 255);
	}

	// Sin upscale: pedir mayor conserva nativo.
	const auto unchanged = TwinpadClient::ScaleRgb(source, 2, 2, 4, 4);
	cemu_assert_debug(unchanged == source);

	// Entrada inválida: buffer que no corresponde a w×h×3 → vacío.
	cemu_assert_debug(TwinpadClient::ScaleRgb(source, 4, 4, 2, 2).empty());
	cemu_assert_debug(TwinpadClient::ScaleRgb(source, 2, 2, 0, 2).empty());

	// SetCaptureResolution: target impar se reduce al par inferior; 0/0 → nativo.
	client.SetCaptureResolution(1919, 1079);
	cemu_assert_debug(client.GetCaptureRequestWidth() == 1918);
	cemu_assert_debug(client.GetCaptureRequestHeight() == 1078);
	client.SetCaptureResolution(0, 0);
	cemu_assert_debug(client.GetCaptureRequestWidth() == 0);
	cemu_assert_debug(client.GetCaptureRequestHeight() == 0);
}
