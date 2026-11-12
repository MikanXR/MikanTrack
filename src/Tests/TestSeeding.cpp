#include "TestCommon.h"

#include "BodyHandHints.h"
#include "HandLandmarkModel.h"
#include "HandSideMatch.h"

// Cross-camera seeding decides whether a projected hint duplicates a hand this
// camera already tracks. That decision used to be split between a LABEL check
// in the vision thread and a POSITIONAL check here, and after a hand leaves
// and re-enters the labels get displaced - so a camera holding one hand under
// the wrong label had one side skipped by the label check and the other
// suppressed by the positional one. Neither was seeded and reacquisition waited
// a full palm-detector interval. The label check is gone; this pins the
// positional one, which now carries the whole decision.
//
// The rest of the seeding path needs the ONNX models, so this covers the part
// that can be tested in isolation.

static int runSeedingTest(const TestArgs& args)
{
	int result= 0;

	auto check = [&result](bool bCondition, const char* label) {
		MIKAN_LOG_INFO("test-seeding") << (bCondition ? "  ok   " : "  FAIL ") << label;
		if (!bCondition)
			result= 1;
	};

	// A 100x80 hand box centred at (300, 200)
	const glm::vec2 boxMin(250.f, 160.f);
	const glm::vec2 boxMax(350.f, 240.f);
	const std::vector<HandBox> oneBox{{boxMin, boxMax}};

	// -- Nothing tracked yet: every hint is worth trying -----
	check(!HandTrackingPipeline::isSeedRedundant(glm::vec2(300.f, 200.f), {}),
		  "no active slots -> not redundant");

	// -- A hint on top of a tracked hand is the camera's own hand -----
	check(HandTrackingPipeline::isSeedRedundant(glm::vec2(300.f, 200.f), oneBox),
		  "hint at the box centre -> redundant");
	check(HandTrackingPipeline::isSeedRedundant(glm::vec2(255.f, 235.f), oneBox),
		  "hint inside the box -> redundant");

	// -- The projection-error margin -----
	// The hint carries the OTHER camera's depth error, which arrives here as a
	// lateral offset, so the box is inflated about its centre before the test.
	// At 1.25 the half-extents grow 50 -> 62.5 and 40 -> 50.
	check(HandTrackingPipeline::isSeedRedundant(glm::vec2(360.f, 200.f), oneBox),
		  "10 px outside the box, inside the margin -> redundant");
	check(HandTrackingPipeline::isSeedRedundant(glm::vec2(300.f, 248.f), oneBox),
		  "8 px below the box, inside the margin -> redundant");
	check(!HandTrackingPipeline::isSeedRedundant(glm::vec2(370.f, 200.f), oneBox),
		  "20 px outside the box, beyond the margin -> seed it");

	// -- A genuinely different hand still gets seeded -----
	check(!HandTrackingPipeline::isSeedRedundant(glm::vec2(700.f, 400.f), oneBox),
		  "far from every box -> seed it");

	// -- Inflation is about the box centre, not the origin -----
	// A box far from the origin would swallow the whole image if the scale
	// were applied to its corners instead.
	const std::vector<HandBox> farBox{{glm::vec2(1000.f, 600.f), glm::vec2(1100.f, 680.f)}};
	check(!HandTrackingPipeline::isSeedRedundant(glm::vec2(300.f, 200.f), farBox),
		  "distant box does not swallow the origin side of the frame");
	check(HandTrackingPipeline::isSeedRedundant(glm::vec2(1050.f, 640.f), farBox),
		  "distant box still matches its own centre");

	// -- Two tracked hands: either one can absorb a hint -----
	const std::vector<HandBox> twoBoxes{{boxMin, boxMax}, {glm::vec2(700.f, 400.f), glm::vec2(800.f, 480.f)}};
	check(HandTrackingPipeline::isSeedRedundant(glm::vec2(750.f, 440.f), twoBoxes),
		  "hint on the second box -> redundant");
	check(!HandTrackingPipeline::isSeedRedundant(glm::vec2(520.f, 320.f), twoBoxes),
		  "hint between the two boxes -> seed it");

	// -- Body hints: the whole-body hands as hints -----
	// A synthetic hand in the MediaPipe order, the wrist at the origin of
	// its own frame and the fingers along +y, placed by an offset
	auto makeHand= [](const glm::vec2& offset) {
		std::array<glm::vec3, WHOLEBODY_HAND_LANDMARK_COUNT> points{};
		const float fingerX[5]= {-30.f, -15.f, 0.f, 12.f, 24.f};
		for (int finger= 0; finger < 5; ++finger)
		{
			for (int joint= 0; joint < 4; ++joint)
			{
				points[1 + finger * 4 + joint]=
					glm::vec3(offset.x + fingerX[finger], offset.y + 40.f + 15.f * (float)joint, 0.f);
			}
		}
		points[0]= glm::vec3(offset, 0.f);
		return points;
	};
	BodyPoseObservation body;
	body.valid= true;
	body.wholeBody.present= true;
	body.wholeBody.hands[0]= makeHand(glm::vec2(900.f, 400.f));
	body.wholeBody.hands[1]= makeHand(glm::vec2(300.f, 400.f));
	body.wholeBody.handScores[0].fill(0.9f);
	body.wholeBody.handScores[1].fill(0.9f);

	std::vector<HandSearchHint> hints;
	std::vector<HandBox> bodyBoxes;
	makeBodyHandHints(body, 1280, 720, hints, bodyBoxes);
	check(hints.size() == 2 && bodyBoxes.size() == 2 && hints[0].bFromBody && hints[0].bHasPalmKeypoints,
		  "two scored whole-body hands -> two body hints and two boxes");
	if (hints.size() == 2)
	{
		// Wrist (900, 400), middle knuckle (900, 440)
		check(hints[0].centerPx == glm::vec2(900.f, 420.f) && hints[0].dirPx == glm::vec2(0.f, 1.f) &&
				  hints[0].palmSizePx == 40.f,
			  "the hint is the palm center, the wrist to middle knuckle direction and length");

		// The seeded crop is the one a tracked slot builds from the landmarks
		const HandRoi seeded= HandRoi::fromPalmDetection(HandTrackingPipeline::makeSeedDetection(hints[0]));
		const HandRoi tracked= HandRoi::fromLandmarks(body.wholeBody.hands[0]);
		check(seeded.palmKeypoints == tracked.palmKeypoints && seeded.palmBoxMin == tracked.palmBoxMin &&
				  seeded.palmBoxMax == tracked.palmBoxMax,
			  "a body hint seeds the ROI the hand's own landmarks give");
	}

	// A hand whose knuckles score under the gate gives no hint and keeps its box
	body.wholeBody.handScores[1][(int)eHandLandmark::INDEX_MCP]= 0.1f;
	makeBodyHandHints(body, 1280, 720, hints, bodyBoxes);
	check(hints.size() == 1 && bodyBoxes.size() == 2, "an unscored hand gives no hint and still protects its box");
	// A hand off the frame gives none either
	body.wholeBody.hands[0]= makeHand(glm::vec2(900.f, 900.f));
	makeBodyHandHints(body, 1280, 720, hints, bodyBoxes);
	check(hints.empty(), "a palm center outside the frame gives no hint");
	makeBodyHandHints(BodyPoseObservation(), 1280, 720, hints, bodyBoxes);
	check(hints.empty() && bodyBoxes.empty(), "no whole-body blocks -> no hints and no boxes");

	// -- Eviction: a body hint with no free slot -----
	// Whole-body hands at x 250..350 and x 850..950, y 380..480
	const std::vector<HandBox> bodyHands{{glm::vec2(250.f, 380.f), glm::vec2(350.f, 480.f)},
										  {glm::vec2(850.f, 380.f), glm::vec2(950.f, 480.f)}};
	const HandBox onLeftHand{glm::vec2(260.f, 390.f), glm::vec2(340.f, 470.f)};
	const HandBox onRightHand{glm::vec2(860.f, 390.f), glm::vec2(940.f, 470.f)};
	const HandBox onTheWall{glm::vec2(600.f, 20.f), glm::vec2(680.f, 100.f)};
	const HandBox alsoOnTheWall{glm::vec2(100.f, 20.f), glm::vec2(180.f, 100.f)};
	// 10 px outside a hand's box, inside its 1.25 margin (12.5 px a side)
	const HandBox besideLeftHand{glm::vec2(360.f, 390.f), glm::vec2(440.f, 470.f)};

	check(HandTrackingPipeline::chooseSlotToEvict({onLeftHand, onTheWall}, {0.9f, 0.9f}, bodyHands) == 1,
		  "the slot on neither whole-body hand gives way");
	check(HandTrackingPipeline::chooseSlotToEvict({onTheWall, onRightHand}, {0.9f, 0.9f}, bodyHands) == 0,
		  "whichever slot index it has");
	check(HandTrackingPipeline::chooseSlotToEvict({onLeftHand, onRightHand}, {0.9f, 0.6f}, bodyHands) == -1,
		  "both slots on whole-body hands -> none gives way");
	check(HandTrackingPipeline::chooseSlotToEvict({besideLeftHand, onRightHand}, {0.9f, 0.9f}, bodyHands) == -1,
		  "a slot within the margin of a whole-body hand is on it");
	check(HandTrackingPipeline::chooseSlotToEvict({onTheWall, alsoOnTheWall}, {0.9f, 0.6f}, bodyHands) == 1,
		  "two slots off both hands -> the lower presence gives way");
	check(HandTrackingPipeline::chooseSlotToEvict({onLeftHand, onTheWall}, {0.9f, 0.9f}, {}) == -1,
		  "no whole-body hand to judge by -> none gives way");

	// -- The hand side by shape: two whole-body hands on one wrist -----
	// The hands share a wrist pixel and differ by their shape alone, the
	// right the mirror of the left about the wrist
	{
		const glm::vec2 wrist(600.f, 400.f);
		auto mirrored= [&](std::array<glm::vec3, WHOLEBODY_HAND_LANDMARK_COUNT> points) {
			for (glm::vec3& point : points)
				point.x= 2.f * wrist.x - point.x;
			return points;
		};
		BodyPoseObservation clasped;
		clasped.valid= true;
		clasped.wholeBody.present= true;
		clasped.wholeBody.hands[0]= makeHand(wrist);
		clasped.wholeBody.hands[1]= mirrored(makeHand(wrist));
		clasped.wholeBody.handScores[0].fill(0.9f);
		clasped.wholeBody.handScores[1].fill(0.9f);
		const WholeBodyExtras& extras= clasped.wholeBody;

		// A slot is the hand model's own reading of a hand, a few pixels off
		auto nudged= [](std::array<glm::vec3, WHOLEBODY_HAND_LANDMARK_COUNT> points) {
			for (glm::vec3& point : points)
				point+= glm::vec3(2.f, -1.5f, 0.f);
			return points;
		};
		check(matchHandSideByShape(nudged(extras.hands[0]), extras).side == 0 &&
				  matchHandSideByShape(nudged(extras.hands[1]), extras).side == 1,
			  "a slot on a wrist both hands share is the hand whose shape it has");

		// Halfway between the two shapes is neither hand's
		std::array<glm::vec3, WHOLEBODY_HAND_LANDMARK_COUNT> between= extras.hands[0];
		for (int index= 0; index < WHOLEBODY_HAND_LANDMARK_COUNT; ++index)
			between[index]= 0.5f * (extras.hands[0][index] + extras.hands[1][index]);
		// A slot needs a palm length, which the average of mirrored hands keeps
		check(matchHandSideByShape(between, extras).side == -1, "a slot no nearer one hand by the margin has no side");

		WholeBodyExtras unscored= extras;
		unscored.handScores[1][(int)eHandLandmark::PINKY_MCP]= 0.1f;
		check(matchHandSideByShape(nudged(extras.hands[0]), unscored).side == -1,
			  "a whole-body hand under the score gate leaves the side undecided");
		check(matchHandSideByShape(nudged(extras.hands[0]), WholeBodyExtras()).side == -1,
			  "no whole-body blocks, no side");
		std::array<glm::vec3, WHOLEBODY_HAND_LANDMARK_COUNT> collapsed;
		collapsed.fill(glm::vec3(wrist, 0.f));
		check(matchHandSideByShape(collapsed, extras).side == -1, "a slot with no size has no side");
	}

	if (result == 0)
		MIKAN_LOG_INFO("test-seeding") << "All seeding checks passed";

	return result;
}

MIKAN_REGISTER_TEST("--test-seeding",
					"Seed redundancy gate, the body model's hand hints, the eviction rule, the hand side by shape",
					eTestCategory::SelfTest, runSeedingTest);
