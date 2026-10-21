#include "Cafe/HW/Latte/Renderer/Renderer.h"
#include "Cafe/Twinpad/TwinpadClient.h"
#include <chrono>
#include "WindowSystem.h"

#include "config/CemuConfig.h"
#include "Cafe/HW/Latte/Core/LatteOverlay.h"

#include <imgui.h>
#include "imgui/imgui_extension.h"
#include <png.h>

#include "config/ActiveSettings.h"

std::unique_ptr<Renderer> g_renderer;

bool Renderer::GetVRAMInfo(int& usageInMB, int& totalInMB) const
{
	usageInMB = totalInMB = -1;
	
#if BOOST_OS_WINDOWS
	if (m_dxgi_wrapper)
	{
		DXGI_QUERY_VIDEO_MEMORY_INFO info{};
		if (m_dxgi_wrapper->QueryVideoMemoryInfo(info))
		{
			totalInMB = (info.Budget / 1000) / 1000;
			usageInMB = (info.CurrentUsage / 1000) / 1000;
			return true;
		}
	}
#endif

	return false;
}


void Renderer::Initialize()
{
	// imgui
	imguiFontAtlas = new ImFontAtlas();
	imguiFontAtlas->AddFontDefault();

	auto setupContext = [](ImGuiContext* context){
		ImGui::SetCurrentContext(context);
		ImGuiIO& io = ImGui::GetIO();
		io.WantSaveIniSettings = false;
		io.IniFilename = nullptr;
	};

	imguiTVContext = ImGui::CreateContext(imguiFontAtlas);
	imguiPadContext = ImGui::CreateContext(imguiFontAtlas);
	setupContext(imguiTVContext);
	setupContext(imguiPadContext);
}

void Renderer::Shutdown()
{
	// imgui
	ImGui::DestroyContext(imguiTVContext);
	ImGui::DestroyContext(imguiPadContext);
    ImGui_ClearFonts();
	delete imguiFontAtlas;
}

bool Renderer::ImguiBegin(bool mainWindow)
{
	sint32 w = 0, h = 0;
	if (mainWindow)
		WindowSystem::GetWindowPhysSize(w, h);
	else if (WindowSystem::IsPadWindowOpen())
		WindowSystem::GetPadWindowPhysSize(w, h);
	else
		return false;
		
	if (w == 0 || h == 0)
		return false;

	// select the right context
	ImGui::SetCurrentContext(mainWindow ? imguiTVContext : imguiPadContext);

	const Vector2f window_size{(float)w, (float)h};
	auto& io = ImGui::GetIO();
	io.DisplaySize = {window_size.x, window_size.y}; // should be only updated in the renderer and only when needed

	ImGui_PrecacheFonts();
	return true;
}

uint8 Renderer::SRGBComponentToRGB(uint8 ci)
{
	const float cs = (float)ci / 255.0f;
	float cl;
	if (cs <= 0.04045)
		cl = cs / 12.92f;
	else
		cl = std::pow((cs + 0.055f) / 1.055f, 2.4f);
	cl = std::min(cl, 1.0f);
	return (uint8)(cl * 255.0f);
}

uint8 Renderer::RGBComponentToSRGB(uint8 cli)
{
	const float cl = (float)cli / 255.0f;
	float cs;
	if (cl < 0.0031308)
		cs = 12.92f * cl;
	else
		cs = 1.055f * std::pow(cl, 0.41666f) - 0.055f;
	cs = std::max(std::min(cs, 1.0f), 0.0f);
	return (uint8)(cs * 255.0f);
}

void Renderer::RequestScreenshot(ScreenshotSaveFunction onSaveScreenshot)
{
	m_screenshot_requested = true;
	m_on_save_screenshot = onSaveScreenshot;
}

void Renderer::CancelScreenshotRequest()
{
	m_screenshot_requested = false;
	m_on_save_screenshot = {};
}

bool Renderer::ShouldCaptureTwinpadFrame() const
{
	if (!m_twinpadCaptureEnabled.load() || !g_renderer || g_renderer.get() != this)
		return false;
	const uint64 now = static_cast<uint64>(std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::steady_clock::now().time_since_epoch()).count());
	const uint64 interval = 1000 / std::max<uint8_t>(TwinpadClient::Instance().GetCaptureFps(), 1);
	const uint64 lastSent = TwinpadClient::Instance().GetLastFrameSentMs();
	if (now - lastSent < interval)
		return false;
	uint64 previous = m_twinpadLastCaptureMs.load();
	if (now - previous < interval)
		return false;
	return m_twinpadLastCaptureMs.compare_exchange_strong(previous, now);
}

void Renderer::CaptureTwinpadFrame(LatteTextureView* texView)
{
	auto& twinpad = TwinpadClient::Instance();
	twinpad.RecordVideoEvent(TwinpadVideoMetricEvent::DRC_CALLBACK);
	if (texView && ShouldCaptureTwinpadFrame())
	{
		twinpad.RecordVideoEvent(TwinpadVideoMetricEvent::CAPTURE_ACCEPTED);
		CaptureTwinpadFrameBackend(texView);
	}
}

void Renderer::PublishTwinpadFrame(const std::vector<uint8>& rgb, int width, int height)
{
	if (m_twinpadCaptureEnabled.load() && width > 0 && height > 0
		&& width <= std::numeric_limits<uint16_t>::max() && height <= std::numeric_limits<uint16_t>::max())
	{
		TwinpadClient::Instance().PublishGamePadFrame(
			std::vector<uint8>(rgb), static_cast<uint16_t>(width), static_cast<uint16_t>(height));
	}
}


void Renderer::SaveScreenshot(const std::vector<uint8>& rgb_data, int width, int height, bool mainWindow)
{
	std::thread(
		[=, screenshotRequested = std::exchange(m_screenshot_requested, false), onSaveScreenshot = std::exchange(m_on_save_screenshot, {})]() {
			if (screenshotRequested && onSaveScreenshot)
			{
				auto notificationMessage = onSaveScreenshot(rgb_data, width, height, mainWindow);
				if (notificationMessage.has_value())
					LatteOverlay_pushNotification(notificationMessage.value(), 2500);
			}
		})
		.detach();
}
