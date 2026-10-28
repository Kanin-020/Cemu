#pragma once

#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

/**
 * Decodificador de `camera_frame` (CR-034). Recibe access units H.264 320x240
 * del teléfono y conserva el último frame en NV12 (YUV 4:2:0) para rellenar la
 * superficie CAM del VPAD.
 */
class TwinpadCameraDecoder
{
public:
	static constexpr uint16_t kWidth = 320;
	static constexpr uint16_t kHeight = 240;

	static TwinpadCameraDecoder& Instance();

	TwinpadCameraDecoder();
	~TwinpadCameraDecoder();

	TwinpadCameraDecoder(const TwinpadCameraDecoder&) = delete;
	TwinpadCameraDecoder& operator=(const TwinpadCameraDecoder&) = delete;

	/** Decodifica una línea `camera_frame` ya validada. Devuelve false si falla. */
	bool DecodeLine(const std::string& line);

	/** Copia el último frame NV12 en `dst` si hay uno. Devuelve bytes copiados. */
	size_t CopyLatestNv12(uint8_t* dst, size_t dstSize);

	/**
	 * Convierte planos YUV420P (con stride) a NV12: Y completo seguido de UV
	 * intercalado. Devuelve false si la geometría no es utilizable.
	 */
	static bool ConvertToNv12(
		const uint8_t* y, int yStride,
		const uint8_t* u, const uint8_t* v, int uvStride,
		uint16_t width, uint16_t height,
		std::vector<uint8_t>& out);

	bool HasFrame() const;

private:
	struct Impl;
	Impl* m_impl{nullptr};
	mutable std::mutex m_mutex;
	std::vector<uint8_t> m_nv12;
};
