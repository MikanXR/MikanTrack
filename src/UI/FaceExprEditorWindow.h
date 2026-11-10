#pragma once

#include <deque>
#include <memory>
#include <string>
#include <vector>

#include "imgui.h"

#include "AvatarMeshView.h"
#include "FaceExprEditTarget.h"
#include "FaceExprInputs.h"
#include "FaceExprTextField.h"
#include "FaceProfile.h"
#include "FaceSources.h"
#include "OrbitCamera.h"

class App;
class AutomationServer;
class AvatarSkeleton;
class FaceProfileBinding;
struct FaceExprCompileError;
class GlFrameBuffer;
struct TrackingFrameResult;

// The expression editor: one window on one row of the active face profile,
// a variable or a VRM expression binding, opened from the row's Edit button
// in the Face tab and moved to another row by the next one or by its own
// header's row picker. Its edits go to the
// profile as the tab's do (the last good binding keeps running while an
// edit does not compile), and Revert restores the text it opened with.
//
// Around the formula field: the inputs the language knows with their live
// values (a click inserts the name), a time trace of the row's value and
// what it reads, a palette of the functions and operators that inserts at
// the caret, and a head close-up of the avatar showing this row's target
// alone (Solo) or the whole profile. Every live value reads the frame the
// scene shows, live or the timeline's replay, as the tab does.
//
// The window closes itself when the row goes away: the active profile
// changes, the row is deleted, or Auto becomes active.
class FaceExprEditorWindow
{
public:
	explicit FaceExprEditorWindow(App* app);
	~FaceExprEditorWindow();

	FaceExprEditorWindow(const FaceExprEditorWindow&)= delete;
	FaceExprEditorWindow& operator=(const FaceExprEditorWindow&)= delete;

	// Opens the editor on a row of the active profile, raising the window.
	// Ignored under Auto or for a row the profile lacks.
	void open(const FaceExprEditTarget& target);
	void close();
	bool isOpen() const { return m_bOpen; }
	// The row the editor is on; null when closed
	const FaceExprEditTarget* getTarget() const { return m_bOpen ? &m_target : nullptr; }

	// Once per frame, open or not: keeps the shown frame for the readouts
	// and draws the window while open. shown is the frame the scene shows.
	void update(const TrackingFrameResult& shown);

	// The automation channel's face namespace: the profile and its rows, and
	// the editor driven without the mouse
	void registerAutomation(AutomationServer* server);

	// What the editor shows, for the automation channel
	struct Status
	{
		bool bOpen= false;
		FaceExprEditTarget target;
		std::string rowLabel;
		std::string text;
		bool bLive= false;
		float value= 0.f;
		std::string error;
		std::vector<std::string> reads;
		bool bSolo= true;
	};
	Status getStatus() const;
	// Replaces the row's text, applying it to the profile at once and to the
	// field on its next draw
	void setText(const std::string& text);
	void revert();
	void setSolo(bool bSolo) { m_bSolo= bSolo; }

private:
	// The per-draw facts every section reads, the tab's rule: the binding
	// has the edited profile's shape (bStructural) and the frame's values
	// came from its generation (bLive)
	struct Frame
	{
		const FaceSources* face= nullptr;
		std::shared_ptr<const FaceProfileBinding> binding;
		const std::vector<FaceExprCompileError>* errors= nullptr;
		bool bStructural= false;
		bool bLive= false;
	};
	struct Series
	{
		// A slot of FaceExprInputs, or a let index when bLet; the result
		// series has index -1
		bool bLet= false;
		int index= -1;
		std::string label;
		ImU32 color= 0;
		// Shown in the plot. Its legend entry toggles it, and the value range
		// fits the shown series.
		bool bOn= true;
	};
	struct Sample
	{
		double timestampMs= 0.0;
		// One per series, in m_series order
		std::vector<float> values;
	};

	Frame makeFrame(const FaceProfile& profile) const;
	bool rowExists(const FaceProfile& profile) const;
	std::string& rowText(FaceProfile& profile) const;
	const std::string& rowText(const FaceProfile& profile) const;
	std::string rowLabel(const FaceProfile& profile) const;
	const FaceExprCompileError* rowError(const Frame& frame, bool bOnName) const;
	// What the row's compiled expression reads directly, from the binding
	// (the last good version while the text does not compile)
	void readReferences(const Frame& frame, std::vector<int>& outInputSlots, std::vector<int>& outLetIndices) const;
	float rowValue(const Frame& frame) const;

	void drawHeader(const FaceProfile& profile, const Frame& frame);
	void drawInputs(const FaceProfile& profile, const Frame& frame);
	bool drawFunction(FaceProfile& profile, const Frame& frame);
	void drawTrace();
	// The lines and the hover readout, between the plot's begin and end
	void drawTracePlot(float yMin, float yMax);
	void drawPalette(const FaceProfile& profile);
	void drawPreview(const Frame& frame);
	// A palette-style button that wraps to the next line when the row is full
	bool paletteButton(const char* label, bool bFirstInRow);
	void queueInsert(const FaceProfile& profile, const std::string& insert);

	// The trace ring: rebuilt when the referenced set changes, fed one
	// sample per new shown frame, trimmed to the window
	void syncSeries(const FaceProfile& profile, const Frame& frame);
	void pushSample(const Frame& frame);
	// Frames the orbit camera on the head
	void frameHead(const AvatarSkeleton& skeleton);
	// The targets the preview drives this frame and their values, Solo or
	// the whole profile; false when there is nothing live to show
	bool previewTargetValues(const Frame& frame, std::vector<float>& outTargetValues) const;

	bool handleAutomation(const std::vector<std::string>& args, std::vector<std::string>& outLines,
						  std::string& outError);

	App* m_app;
	bool m_bOpen= false;
	bool m_bRaiseWindow= false;
	FaceExprEditTarget m_target;
	// The profile the editor opened on, and the text it opened with
	std::string m_profileId;
	std::string m_openedText;
	bool m_bSolo= true;
	// The edit buffer: the active profile, copied over every draw so the
	// widgets can write into it
	FaceProfile m_profile;

	// The frame the scene shows, kept for the readouts between draws
	FaceSources m_shownFace;
	double m_shownTimestampMs= 0.0;
	bool m_bHasShownFrame= false;
	float m_inputValues[FaceExprInputs::k_count]= {};

	FaceExprTextField m_field;
	std::string m_inputFilter;
	// The header's row picker: its search text, and the row picked this draw
	// (index -1 for none)
	std::string m_rowFilter;
	FaceExprEditTarget m_switchTarget;

	// The trace
	std::vector<Series> m_series;
	std::deque<Sample> m_samples;
	double m_lastSampleTimestampMs= -1.0;
	std::vector<int> m_seriesInputSlots;
	std::vector<int> m_seriesLetIndices;
	// The series whose legend entry the mouse was over last draw, -1 for none
	int m_legendHoveredSeries= -1;

	// The preview
	std::unique_ptr<GlFrameBuffer> m_frameBuffer;
	AvatarMeshView m_meshView;
	OrbitCamera m_camera;
	const AvatarSkeleton* m_framedSkeleton= nullptr;
	std::vector<std::vector<float>> m_morphWeights;
	std::vector<float> m_previewTargetValues;
};
