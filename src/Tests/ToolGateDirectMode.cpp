#include "TestCommon.h"

#include "AvatarRetarget.h"
#include "AvatarRetargetMath.h"
#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "BodyPoseSolver.h"
#include "DirectRetarget.h"
#include "DirectStageSolver.h"
#include "FrameRecorder.h"
#include "KalidokitStyleRetarget.h"
#include "SyntheticAvatar.h"

// Direct-mode gate: re-runs the whole-body 3D model over a recording's raw
// frames from one camera and measures what the single-camera direct mode
// would retarget from - the arm segment directions - against the stereo
// rig's output of the same frames. The truth is the faithful replay (the
// triangulated wrists, the IMU forearms where recorded, the body solver's
// shoulders), carried into the camera's own frame, so every comparison is
// one model's view against the rig's view of the same arm.
//
//   --gate-direct <recording.jsonl>... [cameraIndex] [correct] [mirror=<enter>,<leave>] [frames.csv]
//
// correct runs the candidates on the camera's hands with their sides
// corrected from the recorded whole-body hands ahead of the lift, as the
// live direct camera corrects them: the A/B for a take recorded before it
// did, or under a changed rule.
//
// cameraIndex defaults to the recording's body-pose camera. The raw frames
// beside the recording are required (raw frame recording is opt-in). A
// recording without the camera's extrinsics (a direct-mode take) has no
// stereo truth: it runs the retarget candidates alone and reports the two
// truth-free metrics below, with nothing gated.
//
// The model's depth is relative to the hips, so lifting its pixels into
// camera space needs a root depth. Three lifts are measured for each arm:
//   oracle   the root depth that best fits the truth joints, so the number
//            is the model's own direction error with the depth scale solved
//            away
//   live     the root depth from the pixel shoulder width against the
//            configured shoulder width, what the live mode would have
//   planar   the oracle depth with every relative depth zeroed, what a 2D
//            model gives, so the depth axis's contribution is visible
// An elbow flip is a bend vector (elbow off the shoulder-wrist line) pointing
// against the truth's, with both bends past kBendMinMeters.
//
// The retarget comparison runs the avatar solvers on the shared synthetic
// rig: the position retarget over the replayed fused frame is the
// reference, and two candidates run over this camera's recorded hands and
// the re-run observation (with the recording's direct config, neutral or
// not): the direct retarget, and the Kalidokit-style baseline (the webcam
// vtuber tools' pattern, KalidokitStyleRetarget). Against the reference,
// the posed upper-arm and forearm directions are compared in the direct
// world (the reference's carried through the camera's extrinsics and the
// direct conversion), the wrist as the hand's rotation in the forearm, the
// fingers as the proximal bones' rotation in the hand. Two metrics need no
// truth: the posed avatar's shoulder, elbow and wrist projected into the
// camera (the avatar's shoulder midpoint on the model's shoulders at the
// depth the avatar's width implies, the solver's own placement) against
// the model's landmarks, the 2D match the mono mode is for; and the
// rotation of each arm bone between consecutive frames, the jitter. For
// the direct retarget alone: its wrist's distance along the view ray against
// the reference's, the one quantity its scorer chooses, and the hand lift's
// depth scale it measured over the take. All reported, not gated.
//
// The gates cover what the truth measures independently of the model: the
// stereo wrist and the shoulder solved from its own rays and width. The
// truth elbow is not that: without an IMU it comes from the body solver,
// which picks a point on the bone circle by the body-pose camera's own 2D
// elbow, the same landmark under test, so the direction numbers and bend
// flips are reported but not gated until a take with the IMUs worn sets
// them. The limits are from the 2026-10-31 webcam-position take (head in
// frame, shallow tilt, hands raised toward the camera at 0.45 m): shoulders
// 9 px, wrists 58 px, live root depth 7 cm, depth sign 80 percent pooled.

namespace
{
constexpr float kScoreGate= 0.3f;
constexpr float kBendMinMeters= 0.05f;
// A forearm twist that steps this far between two frames is a flip of the
// palm, not a turn of the hand
constexpr float kTwistJumpDegrees= 90.f;
// A twist jump is put down to what changed in the camera's hand over the
// frames before it, this many transitions back: the solver's palm hold shows
// a change up to two frames after the lift made it
constexpr int kTwistJumpCauseFrames= 3;
// A hand the camera has had for fewer frames than this is newly taken
constexpr int kNewHandFrames= 8;

// What a twist jump is put down to, the first that applies
enum class eTwistJumpCause : int
{
	SlotChanged,   // another slot took the side
	MirrorToggled, // the solver took or left the lifted palm's mirror
	PalmarFlipped, // the lift changed which side of the palm plane is palmar
	LeanFlipped,   // the lifted finger axis's lean along the view ray changed sign
	Unexplained,
	Count
};
const char* const kTwistJumpCauseNames[(int)eTwistJumpCause::Count]= {
	"another slot under the side", "the solver's mirror toggled", "the lift's palmar side flipped",
	"the lifted finger axis's lean flipped", "none of these"};
// A wrist this much nearer or farther than the shoulder, by the truth, is a
// depth sign worth asking the model for
constexpr float kDepthSignMinMeters= 0.1f;

constexpr float kShoulderPxP50Gate= 15.f;
constexpr float kWristPxP50Gate= 80.f;
constexpr float kRootDepthErrCmP50Gate= 10.f;
constexpr float kDepthSignAgreePctGate= 75.f;

// The retarget candidates
enum eCandidate
{
	kDirect= 0,
	kBaseline= 1,
	kCandidateCount= 2
};
const char* kCandidateNames[kCandidateCount]= {"direct retarget", "kalidokit-style"};
// A landmark the solvers read: the direct solver's admission rule
constexpr float kSolverMinVisibility= 0.5f;
constexpr float kSolverFrameMarginPx= 16.f;
constexpr float kSolverHandAgreementShoulderWidths= 0.5f;

struct Stats
{
	std::vector<float> values;
	void add(float value) { values.push_back(value); }
	void append(const Stats& other) { values.insert(values.end(), other.values.begin(), other.values.end()); }
	int count() const { return (int)values.size(); }
	float percentile(float fraction) const
	{
		if (values.empty())
			return 0.f;
		std::vector<float> sorted= values;
		std::sort(sorted.begin(), sorted.end());
		const size_t index= std::min(sorted.size() - 1, (size_t)((float)sorted.size() * fraction));
		return sorted[index];
	}
	float p50() const { return percentile(0.5f); }
	float p90() const { return percentile(0.9f); }
};

struct LiftStats
{
	Stats upperArmDeg;
	Stats forearmDeg;
	Stats elbowErrCm;
	Stats wristErrCm;
	int bendFrames= 0;
	int bendFlips= 0;

	void append(const LiftStats& other)
	{
		upperArmDeg.append(other.upperArmDeg);
		forearmDeg.append(other.forearmDeg);
		elbowErrCm.append(other.elbowErrCm);
		wristErrCm.append(other.wristErrCm);
		bendFrames+= other.bendFrames;
		bendFlips+= other.bendFlips;
	}
};

struct SideStats
{
	// Model frames where the truth had a full arm, and how many of those the
	// model provided all three joints on
	int truthArmFrames= 0;
	int modelArmFrames= 0;

	LiftStats oracle;
	LiftStats live;
	LiftStats planar;

	// The model's pixels against the truth joints projected into the
	// camera: the planar agreement, which separates a depth disagreement
	// from a landmark one. The wrist's is the 2D touch-up's residual.
	Stats shoulderPx;
	Stats elbowPx;
	Stats wristPx;
	// Root depth: the live estimate against the oracle
	Stats rootDepthErrCm;
	// The model's wrist depth relative to its shoulder against the truth's,
	// on frames where the truth separates them by kDepthSignMinMeters: does
	// the sign agree, and how much of the difference does the model report
	int depthSignFrames= 0;
	int depthSignAgree= 0;
	Stats wristDepthScale;
	// Direction steps between consecutive model frames, the oracle lift, for
	// the model and for the truth
	Stats modelUpperArmStepDeg;
	Stats modelForearmStepDeg;
	Stats truthUpperArmStepDeg;
	Stats truthForearmStepDeg;
	// The retargeted bones per candidate, see the header comment
	struct Candidate
	{
		// Against the reference
		int frames= 0;
		Stats upperArmDeg;
		Stats forearmDeg;
		Stats wristDeg;
		Stats fingerDeg;
		// Without one: the posed joints against the model's landmarks, and
		// the bones' rotation between consecutive frames
		Stats reprojectShoulderPx;
		Stats reprojectElbowPx;
		Stats reprojectWristPx;
		Stats stepUpperArmDeg;
		Stats stepForearmDeg;
		Stats stepHandDeg;

		void append(const Candidate& other)
		{
			frames+= other.frames;
			upperArmDeg.append(other.upperArmDeg);
			forearmDeg.append(other.forearmDeg);
			wristDeg.append(other.wristDeg);
			fingerDeg.append(other.fingerDeg);
			reprojectShoulderPx.append(other.reprojectShoulderPx);
			reprojectElbowPx.append(other.reprojectElbowPx);
			reprojectWristPx.append(other.reprojectWristPx);
			stepUpperArmDeg.append(other.stepUpperArmDeg);
			stepForearmDeg.append(other.stepForearmDeg);
			stepHandDeg.append(other.stepHandDeg);
		}
	};
	std::array<Candidate, kCandidateCount> candidates{};
	// The direct retarget's wrist along its view ray against the reference's
	// shoulder-to-wrist vector on that ray, cm in the user's units: the
	// magnitude, and signed (positive when the solver put the wrist too far)
	Stats directWristDepthErrCm;
	Stats directWristDepthBiasCm;
	// The inputs behind them: the camera's own palm frame and raw finger
	// angles against the fused ones, so a retarget gap can be read as the
	// single view's or the solver's
	Stats inputPalmDeg;
	// The nearer of the camera's palm and its mirror along the view ray (the
	// lift's wrong depth guess, which the solver undoes by the forearm): what
	// remains is the single view's palm error proper
	Stats inputPalmMirrorDeg;
	Stats inputFingerDeg;
	// The direct solve's forearm twist between consecutive frames that read
	// the hand: how many such pairs, and how many stepped past
	// kTwistJumpDegrees, which a hand does not turn in a frame
	int twistFrames= 0;
	int twistJumps= 0;
	// The jumps by cause, and how many came with the hand turning over in
	// the image (the knuckles' winding changed), with the hand model's own
	// 3D shape changing handedness, and on a newly taken hand
	int twistJumpCauses[(int)eTwistJumpCause::Count]= {};
	int twistJumpsWithImageTurn= 0;
	int twistJumpsWithModelTurn= 0;
	int twistJumpsOnNewHand= 0;
	// The lift's palmar side against the one the hand's side names. Which
	// side of the knuckles' cross product is palmar is fixed by which hand it
	// is: in the lifted camera-space points it points out of the back of a
	// left hand and out of a right hand's palm. Frames with a lifted hand,
	// how many agree, and of the jumps on a palmar flip how many flipped
	// into agreement
	int palmarFrames= 0;
	int palmarAgreeFrames= 0;
	int palmarFlipsToAgree= 0;
	// Frames that read the hand, and how many took the lifted palm's mirror
	int palmFrames= 0;
	int palmMirroredFrames= 0;
	// Frames the hand was on the arm and cut by the frame's edge, its palm
	// and fingers held
	int handCutFrames= 0;

	void append(const SideStats& other)
	{
		palmFrames+= other.palmFrames;
		palmMirroredFrames+= other.palmMirroredFrames;
		handCutFrames+= other.handCutFrames;
		twistFrames+= other.twistFrames;
		twistJumps+= other.twistJumps;
		for (int cause= 0; cause < (int)eTwistJumpCause::Count; ++cause)
			twistJumpCauses[cause]+= other.twistJumpCauses[cause];
		twistJumpsWithImageTurn+= other.twistJumpsWithImageTurn;
		twistJumpsWithModelTurn+= other.twistJumpsWithModelTurn;
		twistJumpsOnNewHand+= other.twistJumpsOnNewHand;
		palmarFrames+= other.palmarFrames;
		palmarAgreeFrames+= other.palmarAgreeFrames;
		palmarFlipsToAgree+= other.palmarFlipsToAgree;
		truthArmFrames+= other.truthArmFrames;
		modelArmFrames+= other.modelArmFrames;
		oracle.append(other.oracle);
		live.append(other.live);
		planar.append(other.planar);
		shoulderPx.append(other.shoulderPx);
		elbowPx.append(other.elbowPx);
		wristPx.append(other.wristPx);
		rootDepthErrCm.append(other.rootDepthErrCm);
		depthSignFrames+= other.depthSignFrames;
		depthSignAgree+= other.depthSignAgree;
		wristDepthScale.append(other.wristDepthScale);
		modelUpperArmStepDeg.append(other.modelUpperArmStepDeg);
		modelForearmStepDeg.append(other.modelForearmStepDeg);
		truthUpperArmStepDeg.append(other.truthUpperArmStepDeg);
		truthForearmStepDeg.append(other.truthForearmStepDeg);
		for (int candidate= 0; candidate < kCandidateCount; ++candidate)
			candidates[candidate].append(other.candidates[candidate]);
		directWristDepthErrCm.append(other.directWristDepthErrCm);
		directWristDepthBiasCm.append(other.directWristDepthBiasCm);
		inputPalmDeg.append(other.inputPalmDeg);
		inputPalmMirrorDeg.append(other.inputPalmMirrorDeg);
		inputFingerDeg.append(other.inputFingerDeg);
	}
};

struct CameraGeometry
{
	glm::dmat4 cameraFromWorld{1.0};
	float fx= 0.f, fy= 0.f, cx= 0.f, cy= 0.f;

	glm::vec3 toCamera(const glm::vec3& world) const
	{
		return glm::vec3(cameraFromWorld * glm::dvec4(glm::dvec3(world), 1.0));
	}
	bool project(const glm::vec3& camera, glm::vec2& outPx) const
	{
		if (camera.z < 1e-3f)
			return false;
		outPx= glm::vec2(fx * camera.x / camera.z + cx, fy * camera.y / camera.z + cy);
		return true;
	}
	// The ray through a pixel, scaled so z = 1
	glm::vec3 rayAtUnitDepth(const glm::vec2& px) const
	{
		return glm::vec3((px.x - cx) / fx, (px.y - cy) / fy, 1.f);
	}
};

// One arm's joints, shoulder, elbow, wrist
struct Arm
{
	glm::vec3 shoulder{0.f};
	glm::vec3 elbow{0.f};
	glm::vec3 wrist{0.f};

	glm::vec3 upperArm() const { return elbow - shoulder; }
	glm::vec3 forearm() const { return wrist - elbow; }
	// The elbow's offset from the shoulder-wrist line
	glm::vec3 bend() const
	{
		const glm::vec3 axis= wrist - shoulder;
		const float axisLength2= glm::dot(axis, axis);
		if (axisLength2 < 1e-9f)
			return glm::vec3(0.f);
		const float t= glm::dot(elbow - shoulder, axis) / axisLength2;
		return elbow - (shoulder + axis * t);
	}
};

// The model's arm: pixel rays and hip-relative depths
struct ModelArm
{
	glm::vec3 ray[3]; // shoulder, elbow, wrist at unit depth
	float relativeDepth[3];
	glm::vec2 px[3];

	Arm lift(float rootDepth, bool bUseDepth) const
	{
		Arm arm;
		glm::vec3* joints[3]= {&arm.shoulder, &arm.elbow, &arm.wrist};
		for (int i= 0; i < 3; ++i)
			*joints[i]= ray[i] * (rootDepth + (bUseDepth ? relativeDepth[i] : 0.f));
		return arm;
	}

	// The root depth that best fits the truth joints, least squares over the
	// three joints: each lifted joint is ray * (root + relative), linear in
	// the root
	float solveRootDepth(const Arm& truth) const
	{
		const glm::vec3* truthJoints[3]= {&truth.shoulder, &truth.elbow, &truth.wrist};
		float numerator= 0.f;
		float denominator= 0.f;
		for (int i= 0; i < 3; ++i)
		{
			numerator+= glm::dot(ray[i], *truthJoints[i]) - glm::dot(ray[i], ray[i]) * relativeDepth[i];
			denominator+= glm::dot(ray[i], ray[i]);
		}
		return denominator > 1e-9f ? numerator / denominator : 0.f;
	}
};

float angleBetweenDeg(const glm::vec3& a, const glm::vec3& b)
{
	const float lengths= glm::length(a) * glm::length(b);
	if (lengths < 1e-9f)
		return 0.f;
	return glm::degrees(acosf(std::max(-1.f, std::min(1.f, glm::dot(a, b) / lengths))));
}

float rotationAngleDeg(const glm::quat& q)
{
	const glm::quat unit= glm::normalize(q);
	return glm::degrees(2.f * acosf(std::min(std::fabs(unit.w), 1.f)));
}

// Both retargets over one frame, their posed bones compared per side
struct RetargetComparison
{
	const AvatarSkeleton* skeleton= nullptr;
	AvatarRetargetConfig referenceConfig;
	DirectRetargetConfig directConfig;
	AvatarRetarget reference;
	DirectStageSolver direct;
	KalidokitStyleRetarget baseline;
	// Marker world to the direct world: through the camera, then the direct
	// conversion at the recording's tilt
	glm::quat directFromMarker{1.f, 0.f, 0.f, 0.f};

	// Each candidate's previous pose, for the step
	std::array<AvatarPose, kCandidateCount> lastPoses{};
	// The input finger comparison's palmar memory per hand
	std::array<HandPoseModel::PalmarSideMemory, 2> inputPalmarMemory{};
	// The direct solve's twist on the last frame that read each hand
	std::array<bool, 2> bLastTwist{};
	std::array<float, 2> lastTwist{};
	// What the camera's hand looked like on the last frames, newest last,
	// for a twist jump's cause
	struct PalmSignals
	{
		bool bValid= false;
		int slotId= -1;
		bool bMirrored= false;
		// Whether the lift's palm normal is along the knuckles' own cross
		// product or against it: the palmar side it chose
		int palmarSign= 0;
		// The sign of the lifted finger axis along the view ray
		int leanSign= 0;
		// The winding of the wrist, index and pinky knuckles in the image
		int imageWinding= 0;
		// Which side of the hand model's own palm plane its thumb is on
		int modelHandedness= 0;
	};
	std::array<std::array<PalmSignals, kTwistJumpCauseFrames + 1>, 2> palmSignals{};
	std::array<int, 2> handFrames{};

	// This frame's truth-free numbers per side and candidate, for the CSV
	struct FrameCandidate
	{
		bool bReproject[3]= {false, false, false}; // shoulder, elbow, wrist
		float reprojectPx[3]= {0.f, 0.f, 0.f};
		bool bStep= false;
		float stepDeg[3]= {0.f, 0.f, 0.f}; // upper arm, forearm, hand
	};
	std::array<std::array<FrameCandidate, kCandidateCount>, 2> frame{};
	// This frame's palm as the direct solve read it per side, for the CSV
	struct FramePalm
	{
		bool bRead= false;
		float twistDeg= 0.f;
		bool bMirrored= false;
		float mirrorGain= 0.f;
		// eTwistJumpCause on a frame the twist jumped, -1 otherwise
		int jumpCause= -1;
	};
	std::array<FramePalm, 2> framePalm{};

	static int signOf(float value) { return value > 0.f ? 1 : (value < 0.f ? -1 : 0); }
	static PalmSignals readPalmSignals(const TrackedHand& hand, const HandPose& pose, bool bMirrored)
	{
		PalmSignals signals;
		if (!hand.tracked || !hand.hasCameraSpace || !pose.tracked)
			return signals;
		signals.bValid= true;
		signals.slotId= hand.slotId;
		signals.bMirrored= bMirrored;
		const int wrist= (int)eHandLandmark::WRIST;
		const int index= (int)eHandLandmark::INDEX_MCP;
		const int pinky= (int)eHandLandmark::PINKY_MCP;
		const int thumb= (int)eHandLandmark::THUMB_MCP;
		const glm::vec3 cameraNormal= glm::cross(hand.cameraPoints[index] - hand.cameraPoints[wrist],
												 hand.cameraPoints[pinky] - hand.cameraPoints[wrist]);
		signals.palmarSign= signOf(glm::dot(pose.palmOrientationCamera * glm::vec3(0.f, 0.f, 1.f), cameraNormal));
		signals.leanSign=
			signOf(glm::dot(pose.palmOrientationCamera * glm::vec3(1.f, 0.f, 0.f), hand.cameraPoints[wrist]));
		const glm::vec2 toIndex= glm::vec2(hand.imagePoints[index]) - glm::vec2(hand.imagePoints[wrist]);
		const glm::vec2 toPinky= glm::vec2(hand.imagePoints[pinky]) - glm::vec2(hand.imagePoints[wrist]);
		signals.imageWinding= signOf(toIndex.x * toPinky.y - toIndex.y * toPinky.x);
		const glm::vec3 modelNormal= glm::cross(hand.modelPoints[index] - hand.modelPoints[wrist],
												hand.modelPoints[pinky] - hand.modelPoints[wrist]);
		signals.modelHandedness= signOf(glm::dot(modelNormal, hand.modelPoints[thumb] - hand.modelPoints[wrist]));
		return signals;
	}

	// bHaveTruth: the fused frame carries the stereo rig's world poses, so
	// the reference can be solved
	void compare(const TrackingFrameResult& fused, bool bHaveTruth, const TrackingFrameResult& cameraInput,
				 const DirectCameraModel& cameraModel, const BodyDimensions& dimensions,
				 std::array<SideStats, 2>& sides)
	{
		using B= eHumanoidBone;
		frame= {};
		AvatarPose referencePose;
		if (bHaveTruth)
			reference.solve(fused, dimensions, *skeleton, referenceConfig, referencePose);
		std::array<AvatarPose, kCandidateCount> poses;
		direct.solve(cameraInput, cameraModel, TrackingFrameResult::HeadPose(), cameraInput.timestampMs, *skeleton,
					 directConfig, poses[kDirect]);
		baseline.solve(cameraInput, cameraModel, *skeleton, directConfig, poses[kBaseline]);
		// The twist's step, between frames that both read the hand
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const DirectRetarget::SolveState::Arm& arm= direct.getSolveState().arms[sideIndex];
			// A hand cut by the frame's edge is held, not read: the step off
			// the held palm when it clears is a reacquire, not a jump
			const bool bRead= arm.bHandSeen && arm.bHandMeasured && !arm.bHandCut;
			framePalm[sideIndex]= FramePalm{bRead, glm::degrees(arm.twist), arm.bPalmMirrored, arm.palmMirrorGain};

			std::array<PalmSignals, kTwistJumpCauseFrames + 1>& history= palmSignals[sideIndex];
			for (int slot= 0; slot < kTwistJumpCauseFrames; ++slot)
				history[slot]= history[slot + 1];
			history[kTwistJumpCauseFrames]=
				readPalmSignals(cameraInput.hands[sideIndex], cameraInput.poses[sideIndex], arm.bPalmMirrored);
			handFrames[sideIndex]= history[kTwistJumpCauseFrames].bValid ? handFrames[sideIndex] + 1 : 0;
			const int sidePalmarSign= sideIndex == (int)eHandSide::Left ? -1 : 1;
			if (history[kTwistJumpCauseFrames].bValid)
			{
				sides[sideIndex].palmarFrames++;
				if (history[kTwistJumpCauseFrames].palmarSign == sidePalmarSign)
					sides[sideIndex].palmarAgreeFrames++;
			}
			sides[sideIndex].handCutFrames+= arm.bHandCut ? 1 : 0;
			if (bRead)
			{
				sides[sideIndex].palmFrames++;
				if (arm.bPalmMirrored)
					sides[sideIndex].palmMirroredFrames++;
			}
			if (bRead && bLastTwist[sideIndex])
			{
				sides[sideIndex].twistFrames++;
				if (fabsf(AvatarRetargetMath::wrapAngle(arm.twist - lastTwist[sideIndex])) > glm::radians(kTwistJumpDegrees))
				{
					sides[sideIndex].twistJumps++;
					bool bSlot= false, bMirror= false, bPalmar= false, bLean= false, bImage= false, bModel= false;
					for (int slot= 0; slot < kTwistJumpCauseFrames; ++slot)
					{
						const PalmSignals& before= history[slot];
						const PalmSignals& after= history[slot + 1];
						if (!before.bValid || !after.bValid)
							continue;
						bSlot|= before.slotId != after.slotId;
						bMirror|= before.bMirrored != after.bMirrored;
						bPalmar|= before.palmarSign != after.palmarSign;
						bLean|= before.leanSign != after.leanSign;
						bImage|= before.imageWinding != after.imageWinding;
						bModel|= before.modelHandedness != after.modelHandedness;
					}
					const eTwistJumpCause cause= bSlot ? eTwistJumpCause::SlotChanged
						: bMirror ? eTwistJumpCause::MirrorToggled
						: bPalmar ? eTwistJumpCause::PalmarFlipped
						: bLean ? eTwistJumpCause::LeanFlipped
						: eTwistJumpCause::Unexplained;
					sides[sideIndex].twistJumpCauses[(int)cause]++;
					if (cause == eTwistJumpCause::PalmarFlipped && history[kTwistJumpCauseFrames].palmarSign == sidePalmarSign)
						sides[sideIndex].palmarFlipsToAgree++;
					sides[sideIndex].twistJumpsWithImageTurn+= bImage ? 1 : 0;
					sides[sideIndex].twistJumpsWithModelTurn+= bModel ? 1 : 0;
					sides[sideIndex].twistJumpsOnNewHand+= handFrames[sideIndex] <= kNewHandFrames ? 1 : 0;
					framePalm[sideIndex].jumpCause= (int)cause;
				}
			}
			bLastTwist[sideIndex]= bRead;
			lastTwist[sideIndex]= arm.twist;
		}
		std::array<AvatarPosedBone, HUMANOID_BONE_COUNT> referencePosed;
		std::array<std::array<AvatarPosedBone, HUMANOID_BONE_COUNT>, kCandidateCount> posed;
		computePosedBones(*skeleton, referencePose, referencePosed);
		for (int candidate= 0; candidate < kCandidateCount; ++candidate)
			computePosedBones(*skeleton, poses[candidate], posed[candidate]);

		// The avatar's shoulder midpoint on the model's shoulders: the rays
		// at the depth the avatar's width implies, in the direct world, the
		// placement the direct solver itself uses
		const BodyPoseObservation& body= cameraInput.body;
		auto usable= [&](int index) {
			if (!body.valid || !body.isProvided(index) || body.visibility[index] < kSolverMinVisibility)
				return false;
			const glm::vec3& point= body.imagePoints[index];
			return point.x >= kSolverFrameMarginPx && point.y >= kSolverFrameMarginPx &&
				point.x <= (float)cameraModel.width - kSolverFrameMarginPx &&
				point.y <= (float)cameraModel.height - kSolverFrameMarginPx;
		};
		const glm::quat worldFromCam= DirectRetarget::worldFromCamera(directConfig.cameraTiltRadians);
		auto rayThrough= [&](int index) {
			const glm::vec3& point= body.imagePoints[index];
			return worldFromCam *
				glm::normalize(glm::vec3((point.x - cameraModel.cx) / cameraModel.fx,
										 (point.y - cameraModel.cy) / cameraModel.fy, 1.f));
		};
		bool bHaveMidpoint= false;
		glm::vec3 midpoint(0.f);
		{
			const int leftIndex= (int)ePoseLandmark::LEFT_SHOULDER;
			const int rightIndex= (int)ePoseLandmark::RIGHT_SHOULDER;
			if (usable(leftIndex) && usable(rightIndex))
			{
				const glm::vec3 leftRay= rayThrough(leftIndex);
				const glm::vec3 rightRay= rayThrough(rightIndex);
				const float depth= BodyPoseSolver::depthFromKnownSeparation(leftRay, rightRay, skeleton->getShoulderWidth());
				if (depth > 0.f)
				{
					midpoint= 0.5f * (leftRay + rightRay) * depth;
					bHaveMidpoint= true;
				}
			}
		}
		const glm::quat camFromWorld= glm::inverse(worldFromCam);
		const float shoulderWidthPx= bHaveMidpoint
			? glm::distance(glm::vec2(body.imagePoints[(int)ePoseLandmark::LEFT_SHOULDER]),
							glm::vec2(body.imagePoints[(int)ePoseLandmark::RIGHT_SHOULDER]))
			: 0.f;

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const B upper= sideIndex == 0 ? B::LeftUpperArm : B::RightUpperArm;
			const B lower= sideIndex == 0 ? B::LeftLowerArm : B::RightLowerArm;
			const B hand= sideIndex == 0 ? B::LeftHand : B::RightHand;
			const int landmarks[3]= {
				sideIndex == 0 ? (int)ePoseLandmark::LEFT_SHOULDER : (int)ePoseLandmark::RIGHT_SHOULDER,
				sideIndex == 0 ? (int)ePoseLandmark::LEFT_ELBOW : (int)ePoseLandmark::RIGHT_ELBOW,
				sideIndex == 0 ? (int)ePoseLandmark::LEFT_WRIST : (int)ePoseLandmark::RIGHT_WRIST,
			};
			// The pixels the joints are measured against: the model's landmarks,
			// the wrist's being the hand model's wrist where the direct solver
			// admits it (tracked, within half a shoulder width of a seen body
			// wrist), since that is the hand the user sees
			glm::vec2 targetPx[3];
			bool bTarget[3];
			for (int i= 0; i < 3; ++i)
			{
				bTarget[i]= usable(landmarks[i]);
				targetPx[i]= glm::vec2(body.imagePoints[landmarks[i]]);
			}
			{
				const TrackedHand& trackedHand= cameraInput.hands[sideIndex];
				const glm::vec2 handWristPx(trackedHand.imagePoints[0]);
				if (trackedHand.tracked && bTarget[2] && shoulderWidthPx > 0.f &&
					glm::distance(handWristPx, targetPx[2]) <= kSolverHandAgreementShoulderWidths * shoulderWidthPx)
					targetPx[2]= handWristPx;
			}
			SideStats& side= sides[sideIndex];
			const bool bReference= bHaveTruth && referencePose.bones[(int)hand].present;

			for (int candidate= 0; candidate < kCandidateCount; ++candidate)
			{
				const AvatarPose& pose= poses[candidate];
				const std::array<AvatarPosedBone, HUMANOID_BONE_COUNT>& candidatePosed= posed[candidate];
				SideStats::Candidate& stats= side.candidates[candidate];
				FrameCandidate& frameStats= frame[sideIndex][candidate];
				if (!pose.bones[(int)hand].present)
					continue;

				// The posed joints into the camera against the model's landmarks
				if (bHaveMidpoint)
				{
					const glm::vec3 posedMidpoint= 0.5f *
						(candidatePosed[(int)B::LeftUpperArm].positionWorld + candidatePosed[(int)B::RightUpperArm].positionWorld);
					const glm::vec3 offset= midpoint - posedMidpoint;
					const B joints[3]= {upper, lower, hand};
					Stats* jointStats[3]= {&stats.reprojectShoulderPx, &stats.reprojectElbowPx, &stats.reprojectWristPx};
					for (int i= 0; i < 3; ++i)
					{
						if (!bTarget[i])
							continue;
						const glm::vec3 inCamera= camFromWorld * (candidatePosed[(int)joints[i]].positionWorld + offset);
						if (inCamera.z < 1e-3f)
							continue;
						const glm::vec2 px(cameraModel.fx * inCamera.x / inCamera.z + cameraModel.cx,
										   cameraModel.fy * inCamera.y / inCamera.z + cameraModel.cy);
						const float error= glm::length(px - targetPx[i]);
						jointStats[i]->add(error);
						frameStats.bReproject[i]= true;
						frameStats.reprojectPx[i]= error;
					}
				}

				// The step from the previous frame's bones
				const AvatarPose& last= lastPoses[candidate];
				if (last.valid && last.bones[(int)hand].present)
				{
					const B bones[3]= {upper, lower, hand};
					Stats* stepStats[3]= {&stats.stepUpperArmDeg, &stats.stepForearmDeg, &stats.stepHandDeg};
					for (int i= 0; i < 3; ++i)
					{
						const float step= rotationAngleDeg(
							glm::inverse(last.bones[(int)bones[i]].deltaWorld) * pose.bones[(int)bones[i]].deltaWorld);
						stepStats[i]->add(step);
						frameStats.stepDeg[i]= step;
					}
					frameStats.bStep= true;
				}

				if (!bReference)
					continue;
				stats.frames++;

				const glm::vec3 referenceUpper= directFromMarker *
					(referencePosed[(int)lower].positionWorld - referencePosed[(int)upper].positionWorld);
				const glm::vec3 referenceForearm= directFromMarker *
					(referencePosed[(int)hand].positionWorld - referencePosed[(int)lower].positionWorld);
				const glm::vec3 candidateUpper=
					candidatePosed[(int)lower].positionWorld - candidatePosed[(int)upper].positionWorld;
				const glm::vec3 candidateForearm=
					candidatePosed[(int)hand].positionWorld - candidatePosed[(int)lower].positionWorld;
				stats.upperArmDeg.add(angleBetweenDeg(referenceUpper, candidateUpper));
				stats.forearmDeg.add(angleBetweenDeg(referenceForearm, candidateForearm));

				// The direct solver's wrist along its view ray against the
				// reference's shoulder-to-wrist vector on the same ray, in the
				// user's units
				if (candidate == kDirect)
				{
					const DirectRetarget::ArmChain& chain= direct.getArmChain(sideIndex);
					const float lengthScale= directConfig.userShoulderWidthMeters > 1e-3f
						? skeleton->getShoulderWidth() / directConfig.userShoulderWidthMeters
						: 1.f;
					if (chain.valid && glm::length(chain.wrist) > 1e-4f && lengthScale > 1e-4f)
					{
						const glm::vec3 ray= glm::normalize(chain.wrist);
						const float referenceAlong=
							glm::dot(directFromMarker * (referencePosed[(int)hand].positionWorld -
														 referencePosed[(int)upper].positionWorld),
									 ray);
						const float errorCm= (glm::dot(chain.wrist - chain.shoulder, ray) - referenceAlong) / lengthScale * 100.f;
						side.directWristDepthBiasCm.add(errorCm);
						side.directWristDepthErrCm.add(std::fabs(errorCm));
					}
				}

				const glm::quat referenceWrist=
					glm::inverse(referencePosed[(int)lower].rotationWorld) * referencePosed[(int)hand].rotationWorld;
				const glm::quat candidateWrist=
					glm::inverse(candidatePosed[(int)lower].rotationWorld) * candidatePosed[(int)hand].rotationWorld;
				stats.wristDeg.add(rotationAngleDeg(glm::inverse(referenceWrist) * candidateWrist));

				float fingerSum= 0.f;
				int fingerCount= 0;
				for (int finger= 0; finger < FINGER_COUNT; ++finger)
				{
					const int proximal= (int)firstHumanoidFingerBone(sideIndex) + finger * 3;
					if (!referencePose.bones[proximal].present || !pose.bones[proximal].present)
						continue;
					const glm::quat referenceLocal=
						glm::inverse(referencePosed[(int)hand].rotationWorld) * referencePosed[proximal].rotationWorld;
					const glm::quat candidateLocal=
						glm::inverse(candidatePosed[(int)hand].rotationWorld) * candidatePosed[proximal].rotationWorld;
					fingerSum+= rotationAngleDeg(glm::inverse(referenceLocal) * candidateLocal);
					fingerCount++;
				}
				if (fingerCount > 0)
					stats.fingerDeg.add(fingerSum / (float)fingerCount);
			}

			// The inputs: the camera's palm frame through the direct conversion
			// against the fused palm carried the same way, and the single view's
			// finger angles as the direct solver measures them against the
			// fused angles (their zeros differ: the fused ones carry the rest
			// capture, so this is a spread, not an error)
			const HandPose& cameraPose= cameraInput.poses[sideIndex];
			const HandPose& fusedPose= fused.poses[sideIndex];
			if (bReference && cameraPose.tracked && cameraPose.hasCameraPose && fusedPose.tracked &&
				fusedPose.hasWorldPose)
			{
				const glm::quat cameraPalm=
					DirectRetarget::worldFromCamera(directConfig.cameraTiltRadians) * cameraPose.palmOrientationCamera;
				const glm::quat fusedPalm= directFromMarker * fusedPose.palmOrientationWorld;
				side.inputPalmDeg.add(rotationAngleDeg(glm::inverse(fusedPalm) * cameraPalm));
				const glm::vec3 viewRay= glm::normalize(
					DirectRetarget::worldFromCamera(directConfig.cameraTiltRadians) * cameraPose.palmPositionCamera);
				const glm::vec3 palmX= cameraPalm * glm::vec3(1.f, 0.f, 0.f);
				const glm::vec3 palmZ= cameraPalm * glm::vec3(0.f, 0.f, 1.f);
				const glm::vec3 mirroredX= palmX - 2.f * glm::dot(palmX, viewRay) * viewRay;
				const glm::vec3 mirroredZ= -(palmZ - 2.f * glm::dot(palmZ, viewRay) * viewRay);
				const glm::quat mirroredPalm= glm::quat_cast(glm::mat3(mirroredX, glm::cross(mirroredZ, mirroredX), mirroredZ));
				side.inputPalmMirrorDeg.add(std::min(rotationAngleDeg(glm::inverse(fusedPalm) * cameraPalm),
													  rotationAngleDeg(glm::inverse(fusedPalm) * mirroredPalm)));
				float angleSum= 0.f;
				std::array<FingerAngles, FINGER_COUNT> monoAngles;
				DirectRetarget::measureMonoFingers(cameraInput.hands[sideIndex], inputPalmarMemory[sideIndex],
												   monoAngles);
				for (int finger= 0; finger < FINGER_COUNT; ++finger)
				{
					const FingerAngles& cameraAngles= monoAngles[finger];
					const FingerAngles& fusedAngles= fusedPose.fingers[finger];
					angleSum+= std::fabs(cameraAngles.proximal - fusedAngles.proximal) +
						std::fabs(cameraAngles.intermediate - fusedAngles.intermediate) +
						std::fabs(cameraAngles.distal - fusedAngles.distal);
				}
				side.inputFingerDeg.add(glm::degrees(angleSum / (float)(FINGER_COUNT * 3)));
			}
		}
		lastPoses= poses;
	}
};

void accumulateLift(const Arm& model, const Arm& truth, LiftStats& stats)
{
	stats.upperArmDeg.add(angleBetweenDeg(model.upperArm(), truth.upperArm()));
	stats.forearmDeg.add(angleBetweenDeg(model.forearm(), truth.forearm()));
	stats.elbowErrCm.add(glm::length(model.elbow - truth.elbow) * 100.f);
	stats.wristErrCm.add(glm::length(model.wrist - truth.wrist) * 100.f);

	const glm::vec3 modelBend= model.bend();
	const glm::vec3 truthBend= truth.bend();
	if (glm::length(modelBend) >= kBendMinMeters && glm::length(truthBend) >= kBendMinMeters)
	{
		stats.bendFrames++;
		if (glm::dot(modelBend, truthBend) < 0.f)
			stats.bendFlips++;
	}
}

int defaultCameraIndex(const AppConfig& config)
{
	for (int cameraIndex= 0; cameraIndex < (int)config.cameraCount(); ++cameraIndex)
	{
		if (config.camera(cameraIndex).bodyPose.enabled)
			return cameraIndex;
	}
	return -1;
}

void logLift(const char* tag, const char* name, const LiftStats& stats)
{
	const float flipPct= stats.bendFrames > 0 ? 100.f * (float)stats.bendFlips / (float)stats.bendFrames : 0.f;
	MIKAN_LOG_INFO(tag) << "    " << name << ": upper arm p50 " << stats.upperArmDeg.p50() << " p90 "
						<< stats.upperArmDeg.p90() << " deg, forearm p50 " << stats.forearmDeg.p50() << " p90 "
						<< stats.forearmDeg.p90() << " deg, elbow p50 " << stats.elbowErrCm.p50() << " cm, wrist p50 "
						<< stats.wristErrCm.p50() << " cm, bend flips " << stats.bendFlips << "/" << stats.bendFrames
						<< " (" << flipPct << "%)";
}

// The retarget candidates: against the reference where there was one, then
// the two truth-free metrics
void logCandidates(const char* tag, const SideStats& side)
{
	for (int candidate= 0; candidate < kCandidateCount; ++candidate)
	{
		const SideStats::Candidate& stats= side.candidates[candidate];
		if (stats.frames > 0)
		{
			MIKAN_LOG_INFO(tag) << "    " << kCandidateNames[candidate] << " vs position retarget on " << stats.frames
								<< " frames: upper arm p50 " << stats.upperArmDeg.p50() << " p90 " << stats.upperArmDeg.p90()
								<< " deg, forearm p50 " << stats.forearmDeg.p50() << " p90 " << stats.forearmDeg.p90()
								<< " deg, wrist p50 " << stats.wristDeg.p50() << " p90 " << stats.wristDeg.p90()
								<< " deg, finger bend p50 " << stats.fingerDeg.p50() << " deg";
		}
		if (stats.stepUpperArmDeg.count() > 0 || stats.reprojectWristPx.count() > 0)
		{
			MIKAN_LOG_INFO(tag) << "    " << kCandidateNames[candidate] << " posed reprojection px p50 (p90): shoulder "
								<< stats.reprojectShoulderPx.p50() << " (" << stats.reprojectShoulderPx.p90() << "), elbow "
								<< stats.reprojectElbowPx.p50() << " (" << stats.reprojectElbowPx.p90() << "), wrist "
								<< stats.reprojectWristPx.p50() << " (" << stats.reprojectWristPx.p90() << ") on "
								<< stats.reprojectShoulderPx.count() << "/" << stats.reprojectElbowPx.count() << "/"
								<< stats.reprojectWristPx.count() << " frames; step deg p50 (p90): upper arm "
								<< stats.stepUpperArmDeg.p50() << " (" << stats.stepUpperArmDeg.p90() << "), forearm "
								<< stats.stepForearmDeg.p50() << " (" << stats.stepForearmDeg.p90() << "), hand "
								<< stats.stepHandDeg.p50() << " (" << stats.stepHandDeg.p90() << ") on "
								<< stats.stepUpperArmDeg.count() << " frames";
		}
	}
	if (side.directWristDepthErrCm.count() > 0)
	{
		MIKAN_LOG_INFO(tag) << "    direct retarget wrist along its ray vs reference: error p50 "
							<< side.directWristDepthErrCm.p50() << " p90 " << side.directWristDepthErrCm.p90()
							<< " cm, signed p50 " << side.directWristDepthBiasCm.p50() << " cm on "
							<< side.directWristDepthErrCm.count() << " frames";
	}
	if (side.twistFrames > 0)
	{
		MIKAN_LOG_INFO(tag) << "    direct retarget twist jumps over " << kTwistJumpDegrees << " deg: " << side.twistJumps
							<< "/" << side.twistFrames << " frame pairs, palm mirrored on " << side.palmMirroredFrames
							<< "/" << side.palmFrames << " frames, palm and fingers held for a hand cut by the frame's edge on "
							<< side.handCutFrames << " frames";
		if (side.twistJumps > 0)
		{
			std::ostringstream causes;
			for (int cause= 0; cause < (int)eTwistJumpCause::Count; ++cause)
				causes << (cause > 0 ? ", " : "") << kTwistJumpCauseNames[cause] << " " << side.twistJumpCauses[cause];
			MIKAN_LOG_INFO(tag) << "      the jumps by what changed in the camera's hand over the " << kTwistJumpCauseFrames
								<< " frames before, the first that applies: " << causes.str();
			MIKAN_LOG_INFO(tag) << "      of the jumps, with the knuckles' winding in the image changing "
								<< side.twistJumpsWithImageTurn << ", with the hand model's 3D shape changing handedness "
								<< side.twistJumpsWithModelTurn << ", on a hand the camera had for " << kNewHandFrames
								<< " frames or fewer " << side.twistJumpsOnNewHand;
		}
	}
	if (side.palmarFrames > 0)
	{
		MIKAN_LOG_INFO(tag) << "    the lift's palmar side is the one the hand's side names on " << side.palmarAgreeFrames << "/"
							<< side.palmarFrames << " lifted frames, and " << side.palmarFlipsToAgree << " of the "
							<< side.twistJumpCauses[(int)eTwistJumpCause::PalmarFlipped]
							<< " jumps on a palmar flip flipped to it";
	}
	if (side.inputPalmDeg.count() > 0)
	{
		MIKAN_LOG_INFO(tag) << "    their inputs, camera vs fused: palm frame p50 " << side.inputPalmDeg.p50()
							<< " p90 " << side.inputPalmDeg.p90() << " deg (nearest mirror p50 "
							<< side.inputPalmMirrorDeg.p50() << " deg), finger angle p50 " << side.inputFingerDeg.p50()
							<< " deg (" << side.inputPalmDeg.count() << " frames)";
	}
}

void logSide(const char* tag, const char* name, const SideStats& side)
{
	MIKAN_LOG_INFO(tag) << "  " << name << ": truth arm on " << side.truthArmFrames
						<< " model frames, model arm on " << side.modelArmFrames;
	if (side.modelArmFrames == 0)
	{
		logCandidates(tag, side);
		return;
	}
	logLift(tag, "oracle", side.oracle);
	logLift(tag, "live  ", side.live);
	logLift(tag, "planar", side.planar);
	MIKAN_LOG_INFO(tag) << "    planar px p50: shoulder " << side.shoulderPx.p50() << ", elbow " << side.elbowPx.p50()
						<< ", wrist " << side.wristPx.p50() << " (p90 " << side.wristPx.p90()
						<< "), live root depth err p50 " << side.rootDepthErrCm.p50() << " p90 "
						<< side.rootDepthErrCm.p90() << " cm (" << side.rootDepthErrCm.count() << " frames)";
	const float signPct=
		side.depthSignFrames > 0 ? 100.f * (float)side.depthSignAgree / (float)side.depthSignFrames : 0.f;
	MIKAN_LOG_INFO(tag) << "    wrist depth against the shoulder: sign agrees on " << side.depthSignAgree << "/"
						<< side.depthSignFrames << " (" << signPct << "%), scale p50 " << side.wristDepthScale.p50();
	MIKAN_LOG_INFO(tag) << "    step between model frames, upper arm model p50 " << side.modelUpperArmStepDeg.p50()
						<< " truth " << side.truthUpperArmStepDeg.p50() << " deg, forearm model "
						<< side.modelForearmStepDeg.p50() << " truth " << side.truthForearmStepDeg.p50() << " deg";
	logCandidates(tag, side);
}

// The columns after recording, frame and side that need the stereo truth
constexpr int kCsvTruthColumns= 12 + 2 * 3 * 3;

void writeCsvHeader(std::ofstream& csv)
{
	csv << "recording,frame,side,rootOracleM,rootLiveM,upperOracleDeg,forearmOracleDeg,upperLiveDeg,forearmLiveDeg,"
		   "upperPlanarDeg,forearmPlanarDeg,elbowOracleCm,shoulderPx,elbowPx,wristPx";
	const char* joints[3]= {"Shoulder", "Elbow", "Wrist"};
	for (const char* source : {"truth", "model"})
		for (const char* joint : joints)
			csv << "," << source << joint << "X," << source << joint << "Y," << source << joint << "Z";
	const char* candidates[kCandidateCount]= {"direct", "baseline"};
	for (const char* candidate : candidates)
		csv << "," << candidate << "ReprojShoulderPx," << candidate << "ReprojElbowPx," << candidate
			<< "ReprojWristPx," << candidate << "StepUpperDeg," << candidate << "StepForearmDeg," << candidate
			<< "StepHandDeg";
	csv << ",directTwistDeg,directPalmMirrored,directPalmMirrorGain,directTwistJumpCause\n";
}

// The direct solve's palm columns: -1 on a frame that did not read the hand
void writeCsvPalm(std::ofstream& csv, const RetargetComparison::FramePalm& palm)
{
	if (palm.bRead)
		csv << "," << palm.twistDeg << "," << (palm.bMirrored ? 1 : 0) << "," << palm.mirrorGain << "," << palm.jumpCause;
	else
		csv << ",-1,-1,-1,-1";
}

// The candidates' columns: a number not measured this frame is written as -1
void writeCsvCandidates(std::ofstream& csv, const std::array<RetargetComparison::FrameCandidate, kCandidateCount>& frame)
{
	for (const RetargetComparison::FrameCandidate& candidate : frame)
	{
		for (int i= 0; i < 3; ++i)
			csv << "," << (candidate.bReproject[i] ? candidate.reprojectPx[i] : -1.f);
		for (int i= 0; i < 3; ++i)
			csv << "," << (candidate.bStep ? candidate.stepDeg[i] : -1.f);
	}
}
} // namespace

static int runGateDirectModeTool(const TestArgs& args)
{
	const char* tag= "gate-direct";

	std::vector<std::string> recordings;
	std::string csvPath;
	int requestedCamera= -1;
	bool bCorrectHandSides= false;
	// The palm mirror's bars for this run in place of the configured ones
	bool bMirrorBars= false;
	float mirrorEnterGain= 0.f;
	float mirrorLeaveGain= 0.f;
	for (const std::string& arg : args)
	{
		if (TrackingRecording::isRecordingFile(arg))
			recordings.push_back(arg);
		else if (arg == "correct")
			bCorrectHandSides= true;
		else if (arg.rfind("mirror=", 0) == 0 && arg.find(',') != std::string::npos)
		{
			bMirrorBars= true;
			mirrorEnterGain= (float)atof(arg.substr(7).c_str());
			mirrorLeaveGain= (float)atof(arg.substr(arg.find(',') + 1).c_str());
		}
		else if (TrackingRecording::isRecordingSidecarFile(arg))
			MIKAN_LOG_INFO(tag) << "Skipping " << arg << ": a take's sidecar, not a take";
		else if (arg.size() > 4 && arg.substr(arg.size() - 4) == ".csv")
			csvPath= arg;
		else
			requestedCamera= atoi(arg.c_str());
	}
	if (recordings.empty())
	{
		MIKAN_LOG_ERROR(tag) << "Usage: --gate-direct <recording.jsonl>... [cameraIndex] [correct] [mirror=<enter>,<leave>] [frames.csv]";
		return 1;
	}
	std::ofstream csv;
	if (!csvPath.empty())
	{
		csv.open(csvPath, std::ios::trunc);
		if (!csv.is_open())
		{
			MIKAN_LOG_ERROR(tag) << "Cannot write " << csvPath;
			return 1;
		}
		writeCsvHeader(csv);
	}

	int result= 0;
	bool bAnyTruth= false;
	std::array<SideStats, 2> pooled{};
	Stats pooledInferenceMs;
	for (const std::string& path : recordings)
	{
		TrackingReplay replay;
		std::string error;
		if (!replay.load(path, error))
		{
			MIKAN_LOG_ERROR(tag) << path << ": load failed: " << error;
			result= 1;
			continue;
		}
		const AppConfig& config= replay.getRecordedConfig();
		const std::string recordingName= std::filesystem::path(path).filename().string();

		const int cameraIndex= requestedCamera >= 0 ? requestedCamera : defaultCameraIndex(config);
		if (cameraIndex < 0 || cameraIndex >= (int)config.cameraCount())
		{
			MIKAN_LOG_ERROR(tag) << path << ": no body-pose camera to default to; pass a cameraIndex";
			result= 1;
			continue;
		}
		const CameraProfile& profile= config.camera(cameraIndex);
		if (!profile.intrinsics.present)
		{
			MIKAN_LOG_ERROR(tag) << path << ": camera " << cameraIndex << " has no intrinsics, nothing to compare in";
			result= 1;
			continue;
		}
		// Without extrinsics there is no stereo truth to carry into the camera:
		// the retarget candidates run alone
		const bool bHaveTruth= profile.extrinsics.present;
		bAnyTruth= bAnyTruth || bHaveTruth;
		const std::string frameDirectory= TrackingRecording::makeFrameDirectoryPath(path);
		if (!std::filesystem::exists(frameDirectory))
		{
			MIKAN_LOG_ERROR(tag) << path << ": no raw frames beside the recording (" << frameDirectory << ")";
			result= 1;
			continue;
		}

		CameraGeometry geometry;
		if (bHaveTruth)
			geometry.cameraFromWorld= glm::inverse(profile.extrinsics.markerFromCamera);
		{
			const MikanMatrix3d& cameraMatrix= profile.intrinsics.intrinsics.undistorted_camera_matrix;
			geometry.fx= (float)cameraMatrix.x0;
			geometry.fy= (float)cameraMatrix.y1;
			geometry.cx= (float)cameraMatrix.z0;
			geometry.cy= (float)cameraMatrix.z1;
		}
		const BodyDimensions dimensions= makeBodyDimensions(config);

		// The retarget comparison on the shared synthetic rig under the
		// recording's own config, default rig settings
		const AvatarModel avatarModel= SyntheticAvatar::makeModel(eVrmVersion::Vrm1, false);
		const AvatarSkeleton skeleton(avatarModel);
		const AvatarRigSettings rigSettings;
		RetargetComparison comparison;
		comparison.skeleton= &skeleton;
		comparison.referenceConfig= makeAvatarRetargetConfig(config, rigSettings);
		comparison.directConfig= makeDirectRetargetConfig(config, rigSettings);
		if (bMirrorBars)
		{
			comparison.directConfig.palmMirrorEnterGain= mirrorEnterGain;
			comparison.directConfig.palmMirrorLeaveGain= mirrorLeaveGain;
			MIKAN_LOG_INFO(tag) << "the palm mirror's bars for this run: enter " << mirrorEnterGain << ", leave "
								<< mirrorLeaveGain;
		}
		comparison.directFromMarker= DirectRetarget::worldFromCamera(comparison.directConfig.cameraTiltRadians) *
			glm::quat_cast(glm::mat3(geometry.cameraFromWorld));
		DirectCameraModel cameraModel;
		cameraModel.fx= geometry.fx;
		cameraModel.fy= geometry.fy;
		cameraModel.cx= geometry.cx;
		cameraModel.cy= geometry.cy;

		BodyPoseTrackerConfig trackerConfig;
		trackerConfig.backend= eBodyPoseBackend::Rtmw3dWholeBody;
		trackerConfig.frameDivider= 1;
		trackerConfig.detectorIntervalFrames= profile.bodyPose.detectorIntervalFrames;
		BodyPoseTracker tracker;
		if (!tracker.load("models", "directml", trackerConfig))
		{
			MIKAN_LOG_ERROR(tag) << "Failed to load the whole-body model (models/rtmw3d_wholebody.onnx)";
			return 1;
		}

		replay.runAll();
		int correctedFrames= 0;
		if (bCorrectHandSides)
		{
			TrackingReplay::WhatIfParams params= replay.makeDefaultWhatIfParams();
			params.correctHandSidesCamera= cameraIndex;
			replay.runWhatIf(params);
		}

		std::array<SideStats, 2> sides{};
		Stats inferenceMs;
		int framesRun= 0;
		int framesMissing= 0;
		// Previous model frame's oracle-lifted arm and truth arm per side,
		// for the step statistics
		bool bHaveLast[2]= {false, false};
		Arm lastModel[2];
		Arm lastTruth[2];

		for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
		{
			const TrackingReplay::ReplayFrame& frame= replay.getFrame(frameIndex);
			const RecordedFrame& recorded= *frame.recorded;
			const RecordedCameraInput* fresh= nullptr;
			for (const RecordedCameraInput& candidate : recorded.freshCameras)
			{
				if (candidate.cameraIndex == cameraIndex)
					fresh= &candidate;
			}
			if (fresh == nullptr)
				continue;

			const cv::Mat image=
				cv::imread(FrameRecorder::makeFramePath(frameDirectory, cameraIndex, fresh->frameIndex),
						   cv::IMREAD_COLOR);
			if (image.empty())
			{
				framesMissing++;
				continue;
			}
			framesRun++;

			BodyPoseObservation observation;
			const auto start= std::chrono::steady_clock::now();
			tracker.process(image, observation);
			inferenceMs.add(
				(float)std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
			// The retargets: the camera's recorded hands with the observation
			// just run, against the replayed fused frame
			bool bCompared= false;
			if (observation.valid && cameraIndex < (int)frame.perCamera.size() && frame.perCameraValid[cameraIndex])
			{
				TrackingFrameResult cameraInput=
					bCorrectHandSides ? frame.whatIfPerCamera[cameraIndex] : frame.perCamera[cameraIndex];
				if (cameraInput.hands[0].bSideCorrected || cameraInput.hands[1].bSideCorrected)
					correctedFrames++;
				cameraInput.body= observation;
				cameraModel.width= cameraInput.frameWidth > 0 ? cameraInput.frameWidth : image.cols;
				cameraModel.height= cameraInput.frameHeight > 0 ? cameraInput.frameHeight : image.rows;
				comparison.compare(frame.replayedFused, bHaveTruth, cameraInput, cameraModel, dimensions, sides);
				bCompared= true;
			}
			if (!bHaveTruth)
			{
				if (bCompared && csv.is_open())
				{
					for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
					{
						csv << recordingName << "," << frameIndex << "," << sideIndex;
						for (int column= 0; column < kCsvTruthColumns; ++column)
							csv << ",-1";
						writeCsvCandidates(csv, comparison.frame[sideIndex]);
						writeCsvPalm(csv, comparison.framePalm[sideIndex]);
						csv << "\n";
					}
				}
				continue;
			}
			if (!observation.valid || !observation.hasDepth)
				continue;

			// The live root depth from the shoulders, shared by both arms
			bool bHaveLiveRoot= false;
			float liveRootDepth= 0.f;
			{
				const int leftSlot= (int)ePoseLandmark::LEFT_SHOULDER;
				const int rightSlot= (int)ePoseLandmark::RIGHT_SHOULDER;
				if (observation.isProvided(leftSlot) && observation.isProvided(rightSlot) &&
					observation.visibility[leftSlot] >= kScoreGate && observation.visibility[rightSlot] >= kScoreGate)
				{
					bHaveLiveRoot= KalidokitStyleRetarget::solveRootDepthFromShoulders(
						geometry.rayAtUnitDepth(glm::vec2(observation.imagePoints[leftSlot])),
						observation.imagePoints[leftSlot].z,
						geometry.rayAtUnitDepth(glm::vec2(observation.imagePoints[rightSlot])),
						observation.imagePoints[rightSlot].z, dimensions.shoulderWidthMeters, liveRootDepth);
				}
			}

			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				SideStats& side= sides[sideIndex];
				const HandPose& truthPose= frame.replayedFused.poses[sideIndex];
				if (!truthPose.tracked || !truthPose.hasWorldPose || !truthPose.hasForearmPose ||
					!truthPose.hasShoulder)
					continue;
				side.truthArmFrames++;

				const int slots[3]= {
					sideIndex == 0 ? (int)ePoseLandmark::LEFT_SHOULDER : (int)ePoseLandmark::RIGHT_SHOULDER,
					sideIndex == 0 ? (int)ePoseLandmark::LEFT_ELBOW : (int)ePoseLandmark::RIGHT_ELBOW,
					sideIndex == 0 ? (int)ePoseLandmark::LEFT_WRIST : (int)ePoseLandmark::RIGHT_WRIST,
				};
				bool bProvided= true;
				ModelArm modelArm;
				for (int i= 0; i < 3; ++i)
				{
					if (!observation.isProvided(slots[i]) || observation.visibility[slots[i]] < kScoreGate)
					{
						bProvided= false;
						break;
					}
					modelArm.ray[i]= geometry.rayAtUnitDepth(glm::vec2(observation.imagePoints[slots[i]]));
					modelArm.relativeDepth[i]= observation.imagePoints[slots[i]].z;
				}
				if (!bProvided)
					continue;
				for (int i= 0; i < 3; ++i)
					modelArm.px[i]= glm::vec2(observation.imagePoints[slots[i]]);
				side.modelArmFrames++;

				Arm truth;
				truth.shoulder= geometry.toCamera(truthPose.shoulderPositionWorld);
				truth.elbow= geometry.toCamera(truthPose.getElbowPositionWorld(dimensions.forearmLengthMeters));
				truth.wrist= geometry.toCamera(truthPose.getWristPositionWorld());

				const float oracleRootDepth= modelArm.solveRootDepth(truth);
				const Arm oracle= modelArm.lift(oracleRootDepth, true);
				const Arm planar= modelArm.lift(oracleRootDepth, false);
				accumulateLift(oracle, truth, side.oracle);
				accumulateLift(planar, truth, side.planar);
				Arm live;
				if (bHaveLiveRoot)
				{
					live= modelArm.lift(liveRootDepth, true);
					accumulateLift(live, truth, side.live);
					side.rootDepthErrCm.add(std::fabs(liveRootDepth - oracleRootDepth) * 100.f);
				}

				// Relative depths lift as root + relative on unit-z rays, so
				// the model's wrist-to-shoulder depth difference is its own,
				// whatever root depth is chosen
				{
					const float truthDepthDelta= truth.wrist.z - truth.shoulder.z;
					const float modelDepthDelta= modelArm.relativeDepth[2] - modelArm.relativeDepth[0];
					if (std::fabs(truthDepthDelta) >= kDepthSignMinMeters)
					{
						side.depthSignFrames++;
						if ((modelDepthDelta < 0.f) == (truthDepthDelta < 0.f))
							side.depthSignAgree++;
						side.wristDepthScale.add(modelDepthDelta / truthDepthDelta);
					}
				}

				float planarPx[3]= {0.f, 0.f, 0.f};
				{
					const glm::vec3* truthJoints[3]= {&truth.shoulder, &truth.elbow, &truth.wrist};
					Stats* planarStats[3]= {&side.shoulderPx, &side.elbowPx, &side.wristPx};
					for (int i= 0; i < 3; ++i)
					{
						glm::vec2 truthPx;
						if (geometry.project(*truthJoints[i], truthPx))
						{
							planarPx[i]= glm::length(modelArm.px[i] - truthPx);
							planarStats[i]->add(planarPx[i]);
						}
					}
				}

				if (bHaveLast[sideIndex])
				{
					side.modelUpperArmStepDeg.add(angleBetweenDeg(oracle.upperArm(), lastModel[sideIndex].upperArm()));
					side.modelForearmStepDeg.add(angleBetweenDeg(oracle.forearm(), lastModel[sideIndex].forearm()));
					side.truthUpperArmStepDeg.add(angleBetweenDeg(truth.upperArm(), lastTruth[sideIndex].upperArm()));
					side.truthForearmStepDeg.add(angleBetweenDeg(truth.forearm(), lastTruth[sideIndex].forearm()));
				}
				bHaveLast[sideIndex]= true;
				lastModel[sideIndex]= oracle;
				lastTruth[sideIndex]= truth;

				if (csv.is_open())
				{
					csv << recordingName << "," << frameIndex << "," << sideIndex << "," << oracleRootDepth << ","
						<< (bHaveLiveRoot ? liveRootDepth : 0.f) << ","
						<< angleBetweenDeg(oracle.upperArm(), truth.upperArm()) << ","
						<< angleBetweenDeg(oracle.forearm(), truth.forearm()) << ","
						<< (bHaveLiveRoot ? angleBetweenDeg(live.upperArm(), truth.upperArm()) : 0.f) << ","
						<< (bHaveLiveRoot ? angleBetweenDeg(live.forearm(), truth.forearm()) : 0.f) << ","
						<< angleBetweenDeg(planar.upperArm(), truth.upperArm()) << ","
						<< angleBetweenDeg(planar.forearm(), truth.forearm()) << ","
						<< glm::length(oracle.elbow - truth.elbow) * 100.f << "," << planarPx[0] << "," << planarPx[1]
						<< "," << planarPx[2];
					const Arm* arms[2]= {&truth, &oracle};
					for (const Arm* arm : arms)
					{
						const glm::vec3* armJoints[3]= {&arm->shoulder, &arm->elbow, &arm->wrist};
						for (const glm::vec3* joint : armJoints)
							csv << "," << joint->x << "," << joint->y << "," << joint->z;
					}
					if (bCompared)
						writeCsvCandidates(csv, comparison.frame[sideIndex]);
					else
						writeCsvCandidates(csv, std::array<RetargetComparison::FrameCandidate, kCandidateCount>{});
					writeCsvPalm(csv, bCompared ? comparison.framePalm[sideIndex] : RetargetComparison::FramePalm{});
					csv << "\n";
				}
			}
		}

		MIKAN_LOG_INFO(tag) << recordingName << " (" << replay.getFrameCount() << " frames, camera " << cameraIndex
							<< ", " << framesRun << " raw frames run, " << framesMissing << " missing, ep "
							<< tracker.activeEp() << ", inference p50 " << inferenceMs.p50() << " ms):";
		if (bCorrectHandSides)
		{
			MIKAN_LOG_INFO(tag) << "  hand sides corrected ahead of the lift: a hand moved on " << correctedFrames
								<< " frames";
		}
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			logSide(tag, sideIndex == 0 ? "Left" : "Right", sides[sideIndex]);
			pooled[sideIndex].append(sides[sideIndex]);
		}
		{
			// The hand lift's depth scale the direct retarget measured over
			// the take, against the length ratio a correct lift would give
			const DirectRetarget::PnpScale left= comparison.direct.getPnpScale(0);
			const DirectRetarget::PnpScale right= comparison.direct.getPnpScale(1);
			const float lengthScale= comparison.directConfig.userShoulderWidthMeters > 1e-3f
				? skeleton.getShoulderWidth() / comparison.directConfig.userShoulderWidthMeters
				: 1.f;
			MIKAN_LOG_INFO(tag) << "  direct retarget hand lift depth scale at the end: left " << left.scale << " ("
								<< left.sampleCount << " samples), right " << right.scale << " (" << right.sampleCount
								<< " samples), length ratio " << lengthScale;
		}
		pooledInferenceMs.append(inferenceMs);
	}

	MIKAN_LOG_INFO(tag) << "Pooled over " << recordings.size() << " recording(s), inference p50 "
						<< pooledInferenceMs.p50() << " ms:";
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		logSide(tag, sideIndex == 0 ? "Left" : "Right", pooled[sideIndex]);

	SideStats all;
	all.append(pooled[0]);
	all.append(pooled[1]);
	logSide(tag, "Both", all);

	// Gates over everything, both sides pooled; a run without stereo truth
	// has reported the truth-free metrics and has nothing to gate
	if (!bAnyTruth)
	{
		MIKAN_LOG_INFO(tag) << "No stereo truth in these recordings: reprojection and step only, nothing gated";
		return result;
	}
	if (all.modelArmFrames == 0)
	{
		MIKAN_LOG_ERROR(tag) << "No arm compared on any frame; nothing to gate";
		return 1;
	}
	const float signPct=
		all.depthSignFrames > 0 ? 100.f * (float)all.depthSignAgree / (float)all.depthSignFrames : 0.f;
	struct Gate
	{
		const char* name;
		float value;
		float limit;
		bool bAtMost;
	};
	const Gate gates[]= {
		{"shoulder px p50", all.shoulderPx.p50(), kShoulderPxP50Gate, true},
		{"wrist px p50", all.wristPx.p50(), kWristPxP50Gate, true},
		{"live root depth err cm p50", all.rootDepthErrCm.p50(), kRootDepthErrCmP50Gate, true},
		{"wrist depth sign agreement %", signPct, kDepthSignAgreePctGate, false},
	};
	bool bAllPassed= true;
	for (const Gate& gate : gates)
	{
		const bool bPassed= gate.bAtMost ? gate.value <= gate.limit : gate.value >= gate.limit;
		bAllPassed= bAllPassed && bPassed;
		MIKAN_LOG_INFO(tag) << "  gate " << gate.name << ": " << gate.value << " (" << (gate.bAtMost ? "at most " : "at least ")
							<< gate.limit << ") " << (bPassed ? "pass" : "FAIL");
	}
	if (!bAllPassed)
	{
		MIKAN_LOG_ERROR(tag) << "Gate(s) failed";
		return 1;
	}
	return result;
}

MIKAN_REGISTER_TEST("--gate-direct", "Whole-body 3D arm directions on one camera against the stereo rig's output",
					eTestCategory::Tool, runGateDirectModeTool);
