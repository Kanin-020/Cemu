#pragma once

#include "Cafe/Twinpad/Core/TwinpadTypes.h"

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <deque>
#include <mutex>
#include <string>
#include <string_view>

// Canal de teclado del modulo Twinpad (CR-044): demanda vigente (texto del
// form en base64url), cola FIFO de transiciones con cap de 8 (drop-old) y
// validacion estricta de keyboard_input. Transplante del cliente original;
// el publisher del cliente consulta los getters de demanda para armar la
// linea y reenviarla tras game_announce.

namespace twinpad_keyboard_channel
{

// CR-044: cap de transiciones pendientes.
constexpr size_t kKeyboardPendingMax = 8;

class TwinpadKeyboardChannel
{
public:
	void SetDemand(bool on, const uint16_t* text, uint16_t count, uint16_t maxLength);
	bool IsKeyboardDemandOn() const;
	uint16_t GetKeyboardDemandMaxLength() const;
	// Snapshot del texto base64url vigente (va con la demanda reenviada).
	std::string GetKeyboardDemandTextBase64Url() const;

	// CR-044: keyboard_input estricto (accion conocida, texto base64url
	// UTF-8 <= 2048 bytes solo con `update`, vacio borra el form). Lo
	// incoherente se ignora (E4).
	bool FeedKeyboardInput(std::string_view line);

	// CR-044: transiciones FIFO para el hilo de render del swkbd; el cap de
	// 8 pendientes suelta las mas antiguas.
	bool ConsumeKeyboardInput(TwinpadKeyboardPending& out);

private:
	std::mutex m_mutex;
	std::atomic<bool> m_demandOn{false};
	std::atomic<uint16_t> m_demandMaxLength{0};
	std::string m_demandTextBase64Url; // protegido por m_mutex
	std::deque<TwinpadKeyboardPending> m_pending;
};

} // namespace twinpad_keyboard_channel
