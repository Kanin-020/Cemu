#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

// Codificacion H.264 del modulo Twinpad: utilidades NAL (start codes, IDR,
// SPS/PPS, contrato de perfil CR-040/CR-041) y los dos backends del cliente
// (hardware FFmpeg CR-039/CR-041 y OpenH264 de fallback). El cliente consume
// interfaz opaca: todos los includes de codecs viven aqui (SRP del CR-058;
// el header del cliente no necesita libavcodec ni wels).

namespace twinpad_h264
{

constexpr uint8_t kH264SpsNalType = 7;
constexpr uint8_t kH264PpsNalType = 8;
constexpr uint8_t kH264IdrNalType = 5;
constexpr uint8_t kAnnexBStartCode[] = {0, 0, 0, 1};
constexpr uint32_t kOpenH264SliceCount = 4;

// ---- Utilidades Annex B (transplante verbatim; CR-055) -------------------
size_t FindAnnexBStartCode(const uint8_t* bytes, size_t size, size_t from, size_t& prefixSize);
bool ContainsH264Nal(const uint8_t* bytes, size_t size, uint8_t nalType);
// CR-040: payload del primer NAL del tipo pedido (sin start code), vacio si no existe.
std::vector<uint8_t> FirstH264Nal(const std::vector<uint8_t>& annexB, uint8_t nalType);
bool ContainsIdrNal(const std::vector<uint8_t>& annexB);

// CR-040: extraccion SPS/PPS Annex B desde extradata AVC (avcC) o Annex B.
bool ExtractH264ParameterSets(const uint8_t* extradata, size_t size, std::vector<uint8_t>& output);

// CR-041: contrato del stream relajado — Baseline (66, con o sin
// constraint_set0) o Main (77), nivel <= 4.2, sin B-frames. Constrained
// Baseline 4.2 (66 + set0 + 42) es subconjunto valido. Motivo: el driver
// radeonsi freeworld no expone Constrained Baseline y el encoder VAAPI
// rechazaba definitivamente el contrato estricto (result=-38).
bool HasH264ContractProfile(const std::vector<uint8_t>& annexB);

// ---- Backend hardware (FFmpeg): handle opaco PIMPL -----------------------
class TwinpadHardwareEncoder
{
public:
	// Intento de creacion del primer candidato disponible (NVENC/QSV/AMF/
	// VAAPI/VideoToolbox); nullptr si ninguno pasa el contrato CR-041.
	static std::unique_ptr<TwinpadHardwareEncoder> TryCreate(uint16_t width, uint16_t height,
		uint8_t fps, uint32_t bitrateKbps);

	explicit TwinpadHardwareEncoder(struct Impl* impl, std::string name, bool nv12, bool hardwareFrames);
	~TwinpadHardwareEncoder();
	TwinpadHardwareEncoder(const TwinpadHardwareEncoder&) = delete;
	TwinpadHardwareEncoder& operator=(const TwinpadHardwareEncoder&) = delete;

	bool IsOpen() const;
	// Nombre del backend elegido (para los logs de descarte/eleccion).
	const std::string& GetName() const { return m_name; }
	// El candidato elegido determina NV12 (hardware frames) o YUV420P.
	bool UsesNv12() const;
	bool HasHardwareFrames() const { return m_hardwareFrames; }
	bool HasBufferedOutput() const { return m_bufferedOutput; }
	uint16_t GetWidth() const;
	uint16_t GetHeight() const;
	uint8_t GetFps() const;
	uint32_t GetBitrateKbps() const;
	// CR-043: el caller pasa el NV12 consolidado (o el RGB? No: el caller
	// convierte con ConvertRgbToYuv antes; aqui solo rutas NV12/Y420P);
	// con EAGAIN devuelve falso con output vacio (frame bufferizado, sale mas tarde).
	bool Encode(const uint8_t* plane0Y, const uint8_t* planeChroma, int width, int height,
		int64_t frameIndex, bool forceIdr, std::vector<uint8_t>& output);

private:
	Impl* m_impl;
	std::string m_name;
	bool m_nv12;
	bool m_hardwareFrames;
	bool m_bufferedOutput{false};
	uint16_t m_width{};
	uint16_t m_height{};
	uint8_t m_fps{};
	uint32_t m_bitrateKbps{};
};

// ---- Backend OpenH264 (fallback CPU): handle opaco ------------------------
void* CreateOpenH264Encoder(uint16_t width, uint16_t height, uint8_t fps, uint32_t bitrateKbps);
void DestroyOpenH264Encoder(void* handle);

} // namespace twinpad_h264
