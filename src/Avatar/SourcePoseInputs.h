#pragma once

#include <string>

#include "ExprInputTable.h"
#include "SourcePose.h"

// The source pose as the inputs a chain's value-or-function setting reads,
// one flat slot table over the source pose the loss stage handed on. This
// table is the one place a source input is added: a slot here, its name in
// the list in the .cpp, its value in fill. The names spell the bones as the
// humanoid table does.
//   source.head.confidence               0
//   source.<side>UpperArm.confidence     1..2, the shoulder's
//   source.<side>LowerArm.confidence     3..4, the forearm's
//   source.<side>Hand.confidence         5..6
//   source.<group>.measured, .carried    7..18, per group (torso, head,
//                                        leftArm, rightArm, leftHand,
//                                        rightHand) in that order
// A confidence the pipeline has none for reads 0. measured is 1 while the
// group is measured on this frame, carried is 1 while it carries anything
// (measured, held, frozen or resting); both are 0 or 1.
namespace SourcePoseInputs
{
constexpr int k_headConfidence= 0;
constexpr int k_upperArmConfidence= 1;   // + eHandSide
constexpr int k_lowerArmConfidence= 3;   // + eHandSide
constexpr int k_handConfidence= 5;       // + eHandSide
constexpr int k_groupStateBase= 7;       // + 2 * group, measured then carried
constexpr int k_groupCount= 6;
constexpr int k_count= k_groupStateBase + 2 * k_groupCount;

// The table a source parameter compiles against
const ExprInputTable& table();

// Writes every slot from the source pose; outInputs holds k_count floats
void fill(const SourcePose& source, float* outInputs);

// The text with each source name of one side spelled for the other:
// "source.left..." becomes "source.right..." and the reverse, matched
// without case, the first letter's case kept. Only the dotted names change;
// numbers, function names and other inputs stay as they are.
std::string swapSides(const std::string& expression);
} // namespace SourcePoseInputs
