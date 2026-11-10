#pragma once

#include <string>

#include "imgui.h"

#include "FaceExprCompiler.h"
#include "FaceProfile.h"

class App;
class FaceProfileBinding;

// The Face tab's row widgets and the text it shares with the expression
// editor, so an error and a target read the same wherever an expression is
// edited.
namespace FaceTabWidgets
{
inline const ImVec4 k_errorColor(1.f, 0.4f, 0.4f, 1.f);
inline const ImVec4 k_warningColor(1.f, 0.8f, 0.4f, 1.f);

// A compile error in the UI's language
std::string errorText(const FaceExprCompileError& error);

// A single-line InputText with its error, if any, underlined at its span and
// spelled out below. True when text changed.
bool textInputWithError(const char* id, std::string& text, const FaceExprCompileError* error);

// Why a binding's output has no target, in the UI's language; "" when it has one
std::string problemText(const FaceProfileBinding& binding, size_t outputIndex);

// A binding's target as its row shows it: the name, with the kind or the mesh
// for a morph target, or the no-target text
std::string targetLabel(const FaceProfileOutput& output);

// The target picker: the avatar's expressions, then each morph name once
// (meaning that morph on every mesh carrying it) and, for a name several
// meshes carry, once per mesh. filter is the picker's search text, kept by
// the caller across frames. True when the output changed.
bool drawTargetPicker(App* app, FaceProfileOutput& output, std::string& filter);
} // namespace FaceTabWidgets
