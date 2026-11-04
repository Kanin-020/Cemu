#pragma once

#include "Cafe/Twinpad/Core/TwinpadTypes.h"

#include <cstdint>
#include <vector>

// Conversion de pixeles del modulo Twinpad (CR-043 + CR-036 + CR-030):
// se encarga de todo el pipeline de color antes del encoder. El cliente ya
// no define la logica; solo consume estos helpers. Toda la aritmetica es la
// misma que la del cliente original (transplante verbatim).

namespace twinpad_yuv
{

// CR-036: conversion sRGB->lineal (bit exacto) con tabla LUT de 256 entradas
// construida en el arranque (nota: se llama desde los formatos *Srgb).
uint8_t SrgbToRgb(uint8_t value);

// CR-030: escalado nearest-neighbor de un frame RGB plano. Sin upscale:
// si target >= nativo se devuelve el buffer de entrada intacto. Devuelve
// vacio si el tamano no corresponde a anchoxalto*3.
std::vector<uint8_t> ScaleRgb(const std::vector<uint8_t>& rgb, uint16_t width,
	uint16_t height, uint16_t targetWidth, uint16_t targetHeight);

// CR-043: conversion RGB8->YUV (I420/NV12) acelerada con SSSE3 cuando el CPU
// lo soporta (runtime dispatch); sin soporte usa el fallback escalar con el
// mismo resultado bit exacto que la original.
// nv12=true intercala UV (una fila de croma: `width` bytes);
// nv12=false escribe planos U y V separados (una fila: width/2 bytes).
void ConvertRgbToYuv(const uint8_t* rgb, int width, int height, bool nv12, std::vector<uint8_t>& dst);

} // namespace twinpad_yuv
