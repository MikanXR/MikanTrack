#pragma once

#include <array>
#include <cstdint>

#include "glm/ext/vector_float3.hpp"

// BlazePose landmark indices (33-point topology). Only the indices the app
// consumes are named; the full set rides through observations and recordings
// so later consumers (VMC output) don't force a schema change.
enum class ePoseLandmark : int
{
	NOSE= 0,
	LEFT_EYE_INNER= 1,
	LEFT_EYE= 2,
	LEFT_EYE_OUTER= 3,
	RIGHT_EYE_INNER= 4,
	RIGHT_EYE= 5,
	RIGHT_EYE_OUTER= 6,
	LEFT_EAR= 7,
	RIGHT_EAR= 8,
	LEFT_SHOULDER= 11,
	RIGHT_SHOULDER= 12,
	LEFT_ELBOW= 13,
	RIGHT_ELBOW= 14,
	LEFT_WRIST= 15,
	RIGHT_WRIST= 16,
	LEFT_HIP= 23,
	RIGHT_HIP= 24,
};

constexpr int POSE_LANDMARK_COUNT= 33;

// BlazePose's own skeleton topology, for overlays. Shared so the live preview
// and the diagnostic dump cannot draw two different skeletons.
constexpr int POSE_CONNECTION_COUNT= 35;
constexpr int POSE_CONNECTIONS[POSE_CONNECTION_COUNT][2]= {
	// face
	{0, 1}, {1, 2}, {2, 3}, {3, 7}, {0, 4}, {4, 5}, {5, 6}, {6, 8}, {9, 10},
	// arms
	{11, 13}, {13, 15}, {15, 17}, {15, 19}, {15, 21}, {17, 19},
	{12, 14}, {14, 16}, {16, 18}, {16, 20}, {16, 22}, {18, 20},
	// torso
	{11, 12}, {11, 23}, {12, 24}, {23, 24},
	// legs
	{23, 25}, {25, 27}, {27, 29}, {27, 31}, {29, 31},
	{24, 26}, {26, 28}, {28, 30}, {28, 32}, {30, 32},
};

// One camera's body-pose result for a frame: the per-camera body-pose stage
// fills it, and it rides TrackingFrameResult through the fusion candidate
// mirrors, recordings, replay, and diagnostics. Landmark sides are
// body-semantic (they name the person's own left/right).
// Where the person box for a model frame came from. Recorded because a
// top-down backend is only as good as its box, and "the detector missed" and
// "the tracked box drifted" fail identically from the outside.
enum class eBodyBoxSource : int
{
	None= 0,
	Detector= 1,   // the person detector fired this model frame
	Tracked= 2,    // grown from the previous model frame's keypoints
	FullFrame= 3,  // nothing else available; the whole image
};

// The whole-body backend's face and hand blocks, which have no slot in the
// 33-slot layout: the 68 iBUG face landmarks and 21 landmarks per hand in the
// MediaPipe hand order.
constexpr int WHOLEBODY_FACE_LANDMARK_COUNT= 68;
constexpr int WHOLEBODY_HAND_LANDMARK_COUNT= 21;

// The iBUG 68 contours as index runs, for overlays: consecutive points of a
// run are joined, and a closed run joins its last point back to its first.
struct WholeBodyFaceContour
{
	int first;
	int last;
	bool bClosed;
};
constexpr int WHOLEBODY_FACE_CONTOUR_COUNT= 9;
constexpr WholeBodyFaceContour WHOLEBODY_FACE_CONTOURS[WHOLEBODY_FACE_CONTOUR_COUNT]= {
	{0, 16, false},  // jaw line
	{17, 21, false}, // right brow
	{22, 26, false}, // left brow
	{27, 30, false}, // nose bridge
	{31, 35, false}, // nose base
	{36, 41, true},  // right eye
	{42, 47, true},  // left eye
	{48, 59, true},  // outer lip
	{60, 67, true},  // inner lip
};

// The face and hand blocks of one whole-body model frame, in the model's own
// orders so the face and hand math can index them directly. The points that
// also have a slot (the eye and mouth corners, three knuckles per hand) are
// repeated here. Sides are the person's own, a hand's side being the arm the
// model attached it to.
struct WholeBodyExtras
{
	// True on an observation from the whole-body backend
	bool present= false;
	// Undistorted full-frame px, z as the slots carry it
	std::array<glm::vec3, WHOLEBODY_FACE_LANDMARK_COUNT> face{};
	// The planar SimCC score per point, clamped to [0,1] as visibility is
	std::array<float, WHOLEBODY_FACE_LANDMARK_COUNT> faceScores{};
	// Indexed by body side, 0 left, 1 right
	std::array<std::array<glm::vec3, WHOLEBODY_HAND_LANDMARK_COUNT>, 2> hands{};
	std::array<std::array<float, WHOLEBODY_HAND_LANDMARK_COUNT>, 2> handScores{};
};

struct BodyPoseObservation
{
	bool valid= false;
	eBodyBoxSource boxSource= eBodyBoxSource::None;

	// Tracker frame index of the model run that produced these landmarks.
	// Divided-out frames re-emit the held observation with this unchanged, so
	// downstream jitter tracking can tell a fresh model result from a re-emit.
	int64_t modelFrameIndex= -1;

	// Landmark-model confidence [0,1]. A gate, not a quality signal: world
	// estimates derive their confidence from measured stability instead.
	float confidence= 0.f;

	// Undistorted full-frame px. z is the landmark's depth relative to the
	// hip midpoint, meters, positive away from the camera, when hasDepth;
	// zero otherwise. A relative depth: the hips' own distance is not in it.
	std::array<glm::vec3, POSE_LANDMARK_COUNT> imagePoints{};
	// True when the backend carries a depth axis (the whole-body 3D model),
	// so imagePoints[].z means something
	bool hasDepth= false;

	// Bit per landmark: which slots this backend actually fills. The 33-slot
	// BlazePose layout is the canonical index space, so a backend with a
	// different topology (RTMPose emits COCO's 17) leaves the rest untouched
	// at the image origin - and an overlay that drew them would pin a phantom
	// landmark in the corner of the frame.
	//
	// 64-bit because there are 33 slots: a 32-bit mask cannot address the
	// last one, and shifting by 33 is undefined rather than merely wrong.
	uint64_t providedMask= 0;
	bool isProvided(int landmarkIndex) const
	{
		return (providedMask & (1ull << landmarkIndex)) != 0;
	}

	// sigmoid [0,1]: within frame and not occluded
	std::array<float, POSE_LANDMARK_COUNT> visibility{};

	// The face and hand blocks, from the whole-body backend only
	WholeBodyExtras wholeBody;
};
