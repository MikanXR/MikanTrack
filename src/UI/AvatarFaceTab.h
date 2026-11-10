#pragma once

#include <memory>
#include <optional>
#include <string>

#include "FaceExprEditTarget.h"
#include "FaceProfile.h"
#include "FaceSources.h"

class App;
class FaceProfileBinding;

// The face profile editor the Profile stage hosts: the avatar's face profiles
// and their VRM expression bindings, in two pieces.
// - The profile bar: the assigned profiles and the active one, assignment
//   from the library, new, duplicate, rename, delete, unassign. Auto is
//   read-only.
// - The rows: the variables and the bindings, each with its function, its
//   compile error marked under the text (or the name), its live value, and
//   an Edit button that opens the row in the expression editor. A binding
//   also has its target picker and its unresolved-target warning.
// Edits go to App::editActiveFaceProfile, which keeps the last good binding
// running while an edit does not compile. Either piece can be drawn more
// than once a frame: the members are edit buffers, and nothing advances
// per frame.
class AvatarFaceTab
{
public:
	void drawProfileBar(App* app);
	// face: the frame the scene shows (live, or the timeline's replay).
	// activeEdit: the row the expression editor is on, drawn highlighted
	// (null: none). Returns the row whose Edit button this draw saw pressed.
	std::optional<FaceExprEditTarget> drawRows(App* app, const FaceSources& face, const FaceExprEditTarget* activeEdit);

private:
	struct Frame;

	bool drawLets(FaceProfile& profile, const Frame& frame);
	bool drawOutputs(App* app, FaceProfile& profile, const Frame& frame);
	// The Edit column of a row: highlights the editor's row, requests an edit
	void drawEditCell(const Frame& frame, bool bLet, int index);

	std::string m_pendingDeleteId;
	std::string m_nameEdit;
	std::string m_nameEditId;
	std::string m_targetFilter;
	// The edit buffer: the active profile, copied over on every draw of the
	// rows so the widgets can write into it. Assigning over the last copy
	// reuses its storage.
	FaceProfile m_profile;
};
