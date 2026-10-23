#include "Common/precompiled.h"
#include "Cafe/Twinpad/TwinpadClient.h"
#include "wxgui/TwinpadGyroMonitorFrame.h"

#include <wx/dcbuffer.h>
#include <wx/settings.h>
#include <wx/sizer.h>

namespace
{
constexpr int kInitialWidth = 640;
constexpr int kInitialHeight = 480;
constexpr int kMinimumWidth = 480;
constexpr int kMinimumHeight = 360;
constexpr int kRefreshIntervalMs = 33;

class TwinpadGyroMonitorView : public wxWindow
{
public:
	explicit TwinpadGyroMonitorView(wxWindow* parent)
		: wxWindow(parent, wxID_ANY)
	{
		SetBackgroundStyle(wxBG_STYLE_PAINT);
		Bind(wxEVT_PAINT, &TwinpadGyroMonitorView::OnPaint, this);
	}

private:
	void OnPaint(wxPaintEvent&)
	{
		wxAutoBufferedPaintDC dc(this);
		dc.SetBackground(wxBrush(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOW)));
		dc.Clear();

		const auto input = TwinpadClient::Instance().GetGyroMonitorState();
		const wxSize size = GetClientSize();
		const int centerX = size.GetWidth() / 2;
		const int graphTop = 24;
		const int graphBottom = std::max(graphTop + 80, size.GetHeight() - 170);
		const int radius = std::max(30, std::min(size.GetWidth() / 2 - 55, (graphBottom - graphTop) / 2 - 12));
		const int centerY = graphTop + (graphBottom - graphTop) / 2;
		const wxRect graphRect(centerX - radius, centerY - radius, radius * 2, radius * 2);

		dc.SetPen(wxPen(wxSystemSettings::GetColour(wxSYS_COLOUR_GRAYTEXT), 2));
		dc.SetBrush(*wxTRANSPARENT_BRUSH);
		dc.DrawRectangle(graphRect);
		dc.SetPen(wxPen(wxSystemSettings::GetColour(wxSYS_COLOUR_GRAYTEXT), 1));
		dc.DrawLine(graphRect.GetLeft() + 8, centerY, graphRect.GetRight() - 8, centerY);
		dc.DrawLine(centerX, graphRect.GetTop() + 8, centerX, graphRect.GetBottom() - 8);

		if (!input.gyroValid)
		{
			dc.SetFont(wxFont(18, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL));
			dc.SetTextForeground(wxSystemSettings::GetColour(wxSYS_COLOUR_GRAYTEXT));
			dc.DrawLabel(wxString::FromUTF8("Esperando entrada de giroscopio…"), graphRect, wxALIGN_CENTER);
			return;
		}

		const auto mapped = TwinpadOrientationMapper::ToCemuTurns(input);
		const std::array<wxColour, 3> colors{wxColour(101, 214, 129), wxColour(255, 91, 91), wxColour(100, 169, 255)};
		const std::array<const char*, 3> names{"Yaw", "Pitch", "Roll"};
		for (size_t axis = 0; axis < names.size(); ++axis)
		{
			const auto endpoint = TwinpadOrientationMapper::ToMonitorEndPoint(input, axis, radius);
			const int endX = centerX + static_cast<int>(endpoint[0]);
			const int endY = centerY + static_cast<int>(endpoint[1]);
			dc.SetPen(wxPen(colors[axis], 5));
			dc.DrawLine(centerX, centerY, endX, endY);
			dc.SetFont(wxFont(14, wxFONTFAMILY_DEFAULT, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_BOLD));
			dc.SetTextForeground(colors[axis]);
			dc.DrawText(names[axis], endX + 5, endY - 20);
		}
		dc.SetPen(*wxTRANSPARENT_PEN);
		dc.SetBrush(wxBrush(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT)));
		dc.DrawCircle(centerX, centerY, 5);

		const int valuesTop = size.GetHeight() - 145;
		dc.SetFont(wxFont(16, wxFONTFAMILY_TELETYPE, wxFONTSTYLE_NORMAL, wxFONTWEIGHT_NORMAL));
		dc.SetTextForeground(wxSystemSettings::GetColour(wxSYS_COLOUR_WINDOWTEXT));
		dc.DrawText(wxString::Format("Android yaw / pitch / roll: %d / %d / %d", input.yaw, input.pitch, input.roll), 18, valuesTop);
		dc.DrawText(wxString::Format("VPAD X / Y / Z turns: %.4f / %.4f / %.4f", mapped[0], mapped[1], mapped[2]), 18, valuesTop + 34);
	}
};
}

TwinpadGyroMonitorFrame::TwinpadGyroMonitorFrame(wxWindow* parent)
	: wxFrame(parent, wxID_ANY, wxString::FromUTF8("Twinpad — Monitor de giroscopio"),
		wxDefaultPosition, wxSize(kInitialWidth, kInitialHeight),
		wxDEFAULT_FRAME_STYLE | wxCLIP_CHILDREN),
	  m_refreshTimer(this)
{
	SetMinClientSize(wxSize(kMinimumWidth, kMinimumHeight));
	m_view = new TwinpadGyroMonitorView(this);
	auto* sizer = new wxBoxSizer(wxVERTICAL);
	sizer->Add(m_view, 1, wxEXPAND);
	SetSizer(sizer);
	Bind(wxEVT_TIMER, &TwinpadGyroMonitorFrame::OnRefresh, this, m_refreshTimer.GetId());
	m_refreshTimer.Start(kRefreshIntervalMs);
}

void TwinpadGyroMonitorFrame::OnRefresh(wxTimerEvent&)
{
	m_view->Refresh(false);
}
