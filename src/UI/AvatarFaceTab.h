#pragma once

#include <memory>
#include <optional>
#include <string>

#include "FaceExprEditTarget.h"
#include "FaceProfile.h"
#include "FaceSources.h"

class App;
class FaceProfileBinding;

// The Avatar panel's Face tab: the avatar's face profiles and their VRM
// expression bindings.
// - The profile bar: the assigned profiles and the active one, assignment
//   from the library, new, duplicate, rename, delete, unassign. Auto is
//   read-only.
// - The variables and the bindings, each with its function, its compile
//   error marked under the text (or the name), its live value, and an Edit
//   button that opens the row in the expression editor. A binding also has
//   its target picker and its unresolved-target warning.
// Edits go to App::editActiveFaceProfile, which keeps the last good binding
// running while an edit does not compile.
class AvatarFaceTab
{
public:
	// face: the frame the scene shows (live, or the timeline's replay).
	// activeEdit: the row the expression editor is on, drawn highlighted
	// (null: none). Returns the row whose Edit button was pressed this frame.
	std::optional<FaceExprEditTarget> draw(App* app, const FaceSources& face, const FaceExprEditTarget* activeEdit);

private:
	struct Frame;

	void drawProfileBar(App* app);
	bool drawLets(FaceProfile& profile, const Frame& frame);
	bool drawOutputs(App* app, FaceProfile& profile, const Frame& frame);
	// The Edit column of a row: highlights the editor's row, requests an edit
	void drawEditCell(const Frame& frame, bool bLet, int index);

	std::string m_pendingDeleteId;
	std::string m_nameEdit;
	std::string m_nameEditId;
	std::string m_targetFilter;
	// The edit buffer: the active profile, copied over every frame so the
	// widgets can write into it. Assigning over the last frame's copy reuses
	// its storage.
	FaceProfile m_profile;
	// The row whose Edit button was pressed this frame
	std::optional<FaceExprEditTarget> m_editRequest;
};
