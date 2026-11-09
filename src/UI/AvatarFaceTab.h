#pragma once

#include <memory>
#include <string>

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
//   error marked under the text (or the name), and its live value. A binding
//   also has its target picker and its unresolved-target warning.
// Edits go to App::editActiveFaceProfile, which keeps the last good binding
// running while an edit does not compile.
class AvatarFaceTab
{
public:
	// face: the frame the scene shows (live, or the timeline's replay)
	void draw(App* app, const FaceSources& face);

private:
	struct Frame;

	void drawProfileBar(App* app);
	bool drawLets(FaceProfile& profile, const Frame& frame);
	bool drawOutputs(App* app, FaceProfile& profile, const Frame& frame);
	bool drawTargetPicker(App* app, FaceProfileOutput& output);

	std::string m_pendingDeleteId;
	std::string m_nameEdit;
	std::string m_nameEditId;
	std::string m_targetFilter;
	// The edit buffer: the active profile, copied over every frame so the
	// widgets can write into it. Assigning over the last frame's copy reuses
	// its storage.
	FaceProfile m_profile;
};
