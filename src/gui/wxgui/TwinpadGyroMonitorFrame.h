#pragma once

#include <wx/frame.h>
#include <wx/timer.h>

class TwinpadGyroMonitorFrame : public wxFrame
{
public:
	explicit TwinpadGyroMonitorFrame(wxWindow* parent);

private:
	void OnRefresh(wxTimerEvent& event);

	wxWindow* m_view{};
	wxTimer m_refreshTimer;
};
