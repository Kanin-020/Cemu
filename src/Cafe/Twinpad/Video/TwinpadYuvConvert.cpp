#include "Cafe/Twinpad/Video/TwinpadYuvConvert.h"

// CR-043: conversion RGB8->YUV acelerada con SSSE3 (runtime dispatch; sin
// soporte se usa el fallback escalar, que conserva el comportamiento exacto
// previo). No cambia el wire: solo el coste por frame del pipeline.
#if defined(__x86_64__) || defined(_M_X64) || defined(__i386__) || defined(_M_IX86)
#define TWINPAD_YUV_X86 1
#include <immintrin.h>
#if defined(_MSC_VER) && !defined(__clang__)
#include <intrin.h>
#endif
#endif

#include <array>
#include <cstring>
#include <algorithm>
#include <cmath>

namespace twinpad_yuv
{
namespace
{

#if defined(TWINPAD_YUV_X86) && (defined(__GNUC__) || defined(__clang__))
#define TWINPAD_YUV_TARGET __attribute__((target("ssse3")))
#else
#define TWINPAD_YUV_TARGET
#endif

bool HasSsse3Support()
{
#ifdef TWINPAD_YUV_X86
#if defined(__GNUC__) || defined(__clang__)
	return __builtin_cpu_supports("ssse3") != 0;
#elif defined(_MSC_VER)
	int cpuInfo[4] = {};
	__cpuid(cpuInfo, 1);
	return (cpuInfo[2] & (1 << 9)) != 0;
#else
	return false;
#endif
#else
	return false;
#endif
}

// Promedio 2x2 de croma para el rango de columnas [columnStart, columnEnd).
// Con nv12=true produce UV intercalado (ancha de fila: width bytes); con
// nv12=false escribe planos U y V separados (una fila: width/2 bytes).
void ConvertChromaScalarRange(const uint8_t* rgb, int width, int height, bool nv12, uint8_t* dst, int columnStart, int columnEnd)
{
	const size_t ySize = static_cast<size_t>(width) * height;
	for (int row = 0; row < height; row += 2)
	{
		const size_t chromaRow = static_cast<size_t>(row / 2) * (nv12 ? width : width / 2);
		for (int column = columnStart; column < columnEnd; column += 2)
		{
			int sumU = 0;
			int sumV = 0;
			for (int dy = 0; dy < 2; ++dy)
			{
				for (int dx = 0; dx < 2; ++dx)
				{
					const size_t source = (static_cast<size_t>(row + dy) * width + column + dx) * 3;
					const int red = rgb[source];
					const int green = rgb[source + 1];
					const int blue = rgb[source + 2];
					sumU += ((-38 * red - 74 * green + 112 * blue + 128) >> 8) + 128;
					sumV += ((112 * red - 94 * green - 18 * blue + 128) >> 8) + 128;
				}
			}
			const uint8_t uValue = static_cast<uint8_t>(std::clamp(sumU / 4, 0, 255));
			const uint8_t vValue = static_cast<uint8_t>(std::clamp(sumV / 4, 0, 255));
			if (nv12)
			{
				dst[ySize + chromaRow + column] = uValue;
				dst[ySize + chromaRow + column + 1] = vValue;
			}
			else
			{
				const size_t chroma = chromaRow + column / 2;
				dst[ySize + chroma] = uValue;
				dst[ySize + ySize / 4 + chroma] = vValue;
			}
		}
	}
}

void ConvertRgbToI420Scalar(const uint8_t* rgb, int width, int height, uint8_t* dst)
{
	for (int row = 0; row < height; ++row)
	{
		for (int column = 0; column < width; ++column)
		{
			const size_t source = (static_cast<size_t>(row) * width + column) * 3;
			const int red = rgb[source];
			const int green = rgb[source + 1];
			const int blue = rgb[source + 2];
			dst[static_cast<size_t>(row) * width + column] = static_cast<uint8_t>(
				((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
		}
	}
	ConvertChromaScalarRange(rgb, width, height, false, dst, 0, width);
}

void ConvertRgbToNv12Scalar(const uint8_t* rgb, int width, int height, uint8_t* dst)
{
	for (int row = 0; row < height; ++row)
	{
		for (int column = 0; column < width; ++column)
		{
			const size_t source = (static_cast<size_t>(row) * width + column) * 3;
			const int red = rgb[source];
			const int green = rgb[source + 1];
			const int blue = rgb[source + 2];
			dst[static_cast<size_t>(row) * width + column] = static_cast<uint8_t>(
				((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
		}
	}
	ConvertChromaScalarRange(rgb, width, height, true, dst, 0, width);
}

#ifdef TWINPAD_YUV_X86
// CR-043: conversion SSSE3. pshufb arma por dword los pares (r,g) (byte B
// aparte) y _mm_madd_epi16 aplica los pares de coeficientes BT.601: un dword
// por pixel en luma y croma. La carga de 16 B sale de la fila cuando
// 3x+15 < 3*width; los guards "x+6 <= width" (luma) y "column+6 <= width"
// (croma) acotan la ventana y las columnas de cola pasan por la ruta escalar.
TWINPAD_YUV_TARGET
static void ConvertRgbToYuvSsse3(const uint8_t* rgb, int width, int height, bool nv12, uint8_t* dst)
{
	const size_t ySize = static_cast<size_t>(width) * height;
	const __m128i zero = _mm_setzero_si128();
	// pshufb reordena BYTE a BYTE con copy directo del indice; el byte fuente
	// 0x80 (indice invalido) escribe 0. Cada dword del vector destino es
	// [r_i, 0, g_i, 0] (o b_i solo); interpretado como dos WORD little-endian:
	// w0 = r_i, w1 = g_i; _mm_madd_epi16 con par (66,129) evalua w0*66 + w1*129
	// (BT.601 completo) en el dword i = 66*r_i + 129*g_i. Coef B (25) en otro
	// madd, con set1 (par constante -> el solo el byte B).
	(void)zero;
	const __m128i rg4Mask = _mm_setr_epi8(0, static_cast<char>(0x80), 1, static_cast<char>(0x80),
		3, static_cast<char>(0x80), 4, static_cast<char>(0x80),
		6, static_cast<char>(0x80), 7, static_cast<char>(0x80),
		9, static_cast<char>(0x80), 10, static_cast<char>(0x80));
	// dword del canal B: [B_i, 0, 0, 0] (no en la posicion 2! si B_{i+1}
	// cayera en ese dword, madd con set1 sumaria dos pixeles).
	const __m128i b4Mask = _mm_setr_epi8(2, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		5, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		8, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		11, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80));
	const __m128i rg2Mask = _mm_setr_epi8(0, static_cast<char>(0x80), 1, static_cast<char>(0x80),
		3, static_cast<char>(0x80), 4, static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80));
	const __m128i b2Mask = _mm_setr_epi8(2, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		5, static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80));
	const __m128i yRGPair = _mm_setr_epi16(66, 129, 66, 129, 66, 129, 66, 129);
	const __m128i uRGPair = _mm_setr_epi16(-38, -74, -38, -74, -38, -74, -38, -74);
	const __m128i vRGPair = _mm_setr_epi16(112, -94, 112, -94, 112, -94, 112, -94);
	const __m128i yBCoeff = _mm_set1_epi16(25);
	const __m128i uBCoeff = _mm_set1_epi16(112);
	const __m128i vBCoeff = _mm_set1_epi16(-18);
	const __m128i roundBias = _mm_set1_epi32(128);
	const __m128i lumaLift = _mm_set1_epi32(16);
	const __m128i chromaLift = _mm_set1_epi32(128);
	// byte bajo del dword i (0x80 = escribe 0): Y queda en [16,235], el byte
	// bajo ES el valor; sin saturacion (packss_epi32 seria SSE4.1).
	const __m128i lowBytesMask = _mm_setr_epi8(0, 4, 8, 12, static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80),
		static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80), static_cast<char>(0x80));

	for (int row = 0; row < height; ++row)
	{
		const uint8_t* line = rgb + static_cast<size_t>(row) * width * 3;
		uint8_t* dstY = dst + static_cast<size_t>(row) * width;
		int x = 0;
		// La ventana de 16 B llega hasta 3x+15; con x+6 <= width queda dentro de la fila.
		for (; x + 6 <= width; x += 4, dstY += 4)
		{
			const __m128i raw = _mm_loadu_si128(reinterpret_cast<const __m128i*>(line + x * 3));
			const __m128i rg = _mm_shuffle_epi8(raw, rg4Mask);
			const __m128i b = _mm_shuffle_epi8(raw, b4Mask);
			__m128i y32 = _mm_add_epi32(_mm_madd_epi16(rg, yRGPair), _mm_madd_epi16(b, yBCoeff));
			y32 = _mm_add_epi32(y32, roundBias);
			y32 = _mm_srli_epi32(y32, 8); // el pre-offset garantiza no negativos
			y32 = _mm_add_epi32(y32, lumaLift);
			// Sin saturacion: Y de 16 a 235 cabe en un byte pese a todo.
			const __m128i yBytes = _mm_shuffle_epi8(y32, lowBytesMask);
			std::memcpy(dstY, &yBytes, 4);
		}
		for (; x < width; ++x, ++dstY)
		{
			const int red = line[x * 3];
			const int green = line[x * 3 + 1];
			const int blue = line[x * 3 + 2];
			*dstY = static_cast<uint8_t>(((66 * red + 129 * green + 25 * blue + 128) >> 8) + 16);
		}
	}

	// Croma SIMD por bloque 2x2; las columnas de cola (x >= width-4) pasan por el
	// helper escalar al final (mismo redondeo).
	if (width >= 6)
	{
		for (int row = 0; row < height; row += 2)
		{
			const uint8_t* rowA = rgb + static_cast<size_t>(row) * width * 3;
			const uint8_t* rowB = rowA + static_cast<size_t>(width) * 3;
			const size_t chromaRow = static_cast<size_t>(row / 2) * (nv12 ? width : width / 2);
			uint8_t* const dstU = dst + ySize + chromaRow;
			uint8_t* const dstV = nv12 ? dstU + 1 : dst + ySize + ySize / 4 + chromaRow;
			for (int column = 0; column < width - 5; column += 2)
			{
				const __m128i rawA = _mm_loadu_si128(reinterpret_cast<const __m128i*>(rowA + column * 3));
				const __m128i rawB = _mm_loadu_si128(reinterpret_cast<const __m128i*>(rowB + column * 3));
				const __m128i rgA = _mm_shuffle_epi8(rawA, rg2Mask);
				const __m128i bA = _mm_shuffle_epi8(rawA, b2Mask);
				const __m128i rgB = _mm_shuffle_epi8(rawB, rg2Mask);
				const __m128i bB = _mm_shuffle_epi8(rawB, b2Mask);
				// u_i/v_i por pixel (BT.601) con el MISMO redondeo por pixel que la
				// ruta escalar: (grpc+128)>>8+128 por FILA (lane0 = (x,y), lane1 =
				// (x+1,y); despues se suma la fila B en las mismas lanes).
				__m128i u32 = _mm_add_epi32(_mm_madd_epi16(rgA, uRGPair), _mm_madd_epi16(bA, uBCoeff));
				u32 = _mm_add_epi32(u32, roundBias);
				u32 = _mm_srai_epi32(u32, 8); // aritmetico: el pre-offset puede ser negativo
				u32 = _mm_add_epi32(u32, chromaLift);
				u32 = _mm_add_epi32(u32, _mm_srai_epi32(_mm_add_epi32(
					_mm_add_epi32(_mm_madd_epi16(rgB, uRGPair), _mm_madd_epi16(bB, uBCoeff)), roundBias), 8));
				u32 = _mm_add_epi32(u32, chromaLift);
				__m128i v32 = _mm_add_epi32(_mm_madd_epi16(rgA, vRGPair), _mm_madd_epi16(bA, vBCoeff));
				v32 = _mm_add_epi32(v32, roundBias);
				v32 = _mm_srai_epi32(v32, 8);
				v32 = _mm_add_epi32(v32, chromaLift);
				v32 = _mm_add_epi32(v32, _mm_srai_epi32(_mm_add_epi32(
					_mm_add_epi32(_mm_madd_epi16(rgB, vRGPair), _mm_madd_epi16(bB, vBCoeff)), roundBias), 8));
				v32 = _mm_add_epi32(v32, chromaLift);
				// Suma horizontal exacta de los 4 valores en lane0: (lane0)+(lane1).
				const __m128i uSum = _mm_add_epi32(u32, _mm_shuffle_epi32(u32, _MM_SHUFFLE(0, 0, 0, 1)));
				const __m128i vSum = _mm_add_epi32(v32, _mm_shuffle_epi32(v32, _MM_SHUFFLE(0, 0, 0, 1)));
				// /4 logico (cada u_i esta en 16..239, la suma en 64..956, no negativa);
				// el dword lane0 cabe en el byte (239 o menos) sin saturacion extra.
				if (nv12)
				{
					dstU[column] = static_cast<uint8_t>(_mm_cvtsi128_si32(_mm_srli_epi32(uSum, 2)));
					dstU[column + 1] = static_cast<uint8_t>(_mm_cvtsi128_si32(_mm_srli_epi32(vSum, 2)));
				}
				else
				{
					dstU[column / 2] = static_cast<uint8_t>(_mm_cvtsi128_si32(_mm_srli_epi32(uSum, 2)));
					dstV[column / 2] = static_cast<uint8_t>(_mm_cvtsi128_si32(_mm_srli_epi32(vSum, 2)));
				}
			}
		}
	}
	ConvertChromaScalarRange(rgb, width, height, nv12, dst, width >= 6 ? width - 4 : 0, width);
}
#endif

} // namespace

uint8_t SrgbToRgb(uint8_t value)
{
	static const std::array<uint8_t, 256> linearLookup = [] {
		std::array<uint8_t, 256> lookup{};
		for (size_t index = 0; index < lookup.size(); ++index)
		{
			const float srgb = static_cast<float>(index) / 255.0f;
			const float linear = srgb <= 0.04045f ? srgb / 12.92f : std::pow((srgb + 0.055f) / 1.055f, 2.4f);
			lookup[index] = static_cast<uint8_t>(std::min(linear, 1.0f) * 255.0f);
		}
		return lookup;
	}();
	return linearLookup[value];
}

std::vector<uint8_t> ScaleRgb(const std::vector<uint8_t>& rgb, uint16_t width, uint16_t height, uint16_t targetWidth, uint16_t targetHeight) // CR-030
{
	if (!width || !height || !targetWidth || !targetHeight || rgb.size() != static_cast<size_t>(width) * height * 3)
		return {};
	// CR-074: nearest-neighbor en ambas direcciones; identidad solo si el
	// target coincide (devuelve el buffer sin copiar).
	if (targetWidth == width && targetHeight == height)
		return rgb;
	std::vector<uint8_t> target(static_cast<size_t>(targetWidth) * targetHeight * 3);
	for (uint32_t targetY = 0; targetY < targetHeight; ++targetY)
	{
		const size_t sourceY = targetY * height / targetHeight;
		for (uint32_t targetX = 0; targetX < targetWidth; ++targetX)
		{
			const size_t sourceX = targetX * width / targetWidth;
			const size_t source = (sourceY * width + sourceX) * 3;
			const size_t destination = (targetY * targetWidth + targetX) * 3;
			target[destination] = rgb[source];
			target[destination + 1] = rgb[source + 1];
			target[destination + 2] = rgb[source + 2];
		}
	}
	return target;
}

void ConvertRgbToYuv(const uint8_t* rgb, int width, int height, bool nv12, std::vector<uint8_t>& dst)
{
	dst.resize(static_cast<size_t>(width) * height * 3 / 2);
#ifdef TWINPAD_YUV_X86
	static const bool hasSsse3 = HasSsse3Support();
	if (hasSsse3)
	{
		ConvertRgbToYuvSsse3(rgb, width, height, nv12, dst.data());
		return;
	}
#endif
	if (nv12)
		ConvertRgbToNv12Scalar(rgb, width, height, dst.data());
	else
		ConvertRgbToI420Scalar(rgb, width, height, dst.data());
}

} // namespace twinpad_yuv
