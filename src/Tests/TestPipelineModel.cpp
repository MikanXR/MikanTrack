#include "TestCommon.h"

#include <functional>
#include <set>

#include "PipelineModel.h"

// The pipeline model: the layout per project shape, the stage ids' text
// form, and the status rules over hand-built inputs. The last cases walk a
// grid of faults over every project shape and require that every status text
// the string table carries is reached by some input and that every text the
// rules return is in the table, so a rule and its string cannot drift apart.

namespace
{
using S= ePipelineStage;
using Inputs= PipelineStatusInputs;

const char* kTag= "test-pipeline-model";

PipelineStageId id(S stage, int cameraIndex= -1)
{
	PipelineStageId out;
	out.stage= stage;
	out.cameraIndex= cameraIndex;
	return out;
}

// Everything present, calibrated and tracking
Inputs makeHealthy(bool bDirect, int cameraCount, bool bImu, int bodyCamera)
{
	Inputs in;
	in.bDirect= bDirect;
	in.cameraCount= cameraCount;
	in.path= bDirect ? Inputs::ePath::Direct : Inputs::ePath::Fused;
	for (int cameraIndex= 0; cameraIndex < cameraCount; ++cameraIndex)
	{
		Inputs::Camera& camera= in.cameras[cameraIndex];
		camera.bDeviceOpen= true;
		camera.bStreaming= true;
		camera.captureFps= 30.f;
		camera.bHandPipelineReady= true;
		camera.bIntrinsics= true;
		camera.bExtrinsics= !bDirect;
		camera.handsTracked= 2;
		if (cameraIndex == bodyCamera)
		{
			camera.bBodyEnabled= true;
			camera.bBodyModelLoaded= true;
			camera.bBodyValid= true;
			camera.bodyConfidence= 0.85f;
			camera.bBodyHasDepth= bDirect;
		}
	}
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		Inputs::Hand& hand= in.hands[sideIndex];
		hand.bTracked= true;
		hand.bWorldPose= !bDirect;
		hand.confidence= 0.9f;
		hand.dominantCamera= sideIndex % std::max(cameraCount, 1);
		hand.bForearmPose= !bDirect && (bImu || bodyCamera >= 0);
		hand.forearmConfidence= 0.8f;
		hand.bShoulder= !bDirect && bodyCamera >= 0;
		in.bArmPlaced[sideIndex]= true;

		Inputs::Imu& imu= in.imu[sideIndex];
		imu.bConnected= bImu;
		imu.bStreaming= bImu;
		imu.bCalibrated= bImu;
		imu.bOrientationValid= bImu;
		imu.sampleRateHz= bImu ? 200.f : 0.f;

		Inputs::DirectArm& arm= in.directArms[sideIndex];
		arm.bChain= bDirect;
		arm.bHandSeen= bDirect;
		arm.bElbowSeen= bDirect;
		arm.bHandMeasured= bDirect;
	}
	in.bHeadValid= bodyCamera >= 0;
	in.bImuEnabled= bImu;
	in.bDirectIntrinsics= bDirect;
	in.bDirectNeutral= bDirect;
	in.bDirectTorso= bDirect;
	in.directShoulderDepthMeters= 0.62f;
	in.bAvatarLoaded= true;
	in.bOscEnabled= true;
	in.bOscRunning= true;
	in.oscSendRateHz= 60.f;
	in.oscMinConfidence= 0.3f;
	in.oscPort= 39539;
	in.bFaceEnabled= true;
	in.bFaceBound= true;
	in.bFaceStreaming= true;
	in.bFaceTracked= true;
	in.faceDatagramsPerSecond= 60.f;
	// The webcam face is the direct camera's
	in.bWebcamPresent= in.bDirect;
	in.bWebcamTracked= in.bDirect;
	in.bWebcamNeutral= in.bDirect;
	in.bAudioEnabled= true;
	in.bAudioDeviceOpen= true;
	in.bAudioBackendReady= true;
	in.bAudioGateOpen= true;
	in.audioLevelDbfs= -20.f;
	in.profileTargetCount= 52;
	return in;
}

eStageState stateOf(const Inputs& in, const PipelineStageId& stage)
{
	return PipelineModel::computeStatus(in, stage).state;
}

std::string valueOf(const Inputs& in, const PipelineStageId& stage)
{
	return PipelineModel::computeStatus(in, stage).value.key;
}

std::string reasonOf(const Inputs& in, const PipelineStageId& stage)
{
	return PipelineModel::computeStatus(in, stage).reason.key;
}

// One fault applied to a healthy project
struct Fault
{
	const char* name;
	std::function<void(Inputs&)> apply;
};

std::vector<Fault> makeFaults()
{
	return {
		{"none", [](Inputs&) {}},
		{"no cameras", [](Inputs& in) { in.cameraCount= 0; }},
		{"camera device closed", [](Inputs& in) { in.cameras[0].bDeviceOpen= false; }},
		{"camera stream stopped", [](Inputs& in) { in.cameras[0].bStreaming= false; }},
		{"tracking paused", [](Inputs& in) { in.cameras[0].bTrackingPaused= true; }},
		{"hand pipeline failed", [](Inputs& in) { in.cameras[0].bHandPipelineReady= false; }},
		{"no intrinsics", [](Inputs& in) {
			 in.cameras[0].bIntrinsics= false;
			 in.bDirectIntrinsics= false;
		 }},
		{"one camera without extrinsics", [](Inputs& in) { in.cameras[0].bExtrinsics= false; }},
		{"no extrinsics", [](Inputs& in) {
			 for (Inputs::Camera& camera : in.cameras)
				 camera.bExtrinsics= false;
			 if (!in.bDirect)
				 in.path= Inputs::ePath::Passthrough;
		 }},
		{"body camera without extrinsics", [](Inputs& in) {
			 for (Inputs::Camera& camera : in.cameras)
			 {
				 if (camera.bBodyEnabled)
					 camera.bExtrinsics= false;
			 }
		 }},
		{"body model missing", [](Inputs& in) {
			 for (Inputs::Camera& camera : in.cameras)
				 camera.bBodyModelLoaded= false;
		 }},
		{"nobody in view", [](Inputs& in) {
			 for (Inputs::Camera& camera : in.cameras)
			 {
				 camera.bBodyValid= false;
				 camera.handsTracked= 0;
			 }
			 for (Inputs::Hand& hand : in.hands)
			 {
				 hand.bTracked= false;
				 hand.bShoulder= false;
			 }
			 in.bHeadValid= false;
			 in.bDirectTorso= false;
			 for (Inputs::DirectArm& arm : in.directArms)
				 arm= Inputs::DirectArm();
			 in.bArmPlaced[0]= in.bArmPlaced[1]= false;
		 }},
		{"one hand lost", [](Inputs& in) {
			 in.hands[0].bTracked= false;
			 in.directArms[0].bChain= false;
			 in.directArms[0].bResting= true;
			 in.directArms[0].restBlend= 0.4f;
		 }},
		{"both arms at rest", [](Inputs& in) {
			 for (Inputs::DirectArm& arm : in.directArms)
			 {
				 arm.bChain= false;
				 arm.bResting= true;
				 arm.restBlend= 1.f;
			 }
		 }},
		{"no camera frames", [](Inputs& in) { in.path= Inputs::ePath::Idle; }},
		{"imu disconnected", [](Inputs& in) { in.imu[0].bConnected= in.imu[1].bConnected= false; }},
		{"one imu disconnected", [](Inputs& in) { in.imu[1].bConnected= false; }},
		{"imu uncalibrated", [](Inputs& in) { in.imu[0].bCalibrated= false; }},
		{"imu silent", [](Inputs& in) { in.imu[0].bStreaming= false; }},
		{"imu converging", [](Inputs& in) { in.imu[1].bOrientationValid= false; }},
		{"replay", [](Inputs& in) { in.bReplay= true; }},
		{"replay of a take with no stage state", [](Inputs& in) {
			 in.bReplay= true;
			 in.bStageStateRecorded= false;
		 }},
		{"no neutral", [](Inputs& in) { in.bDirectNeutral= false; }},
		{"no neutral, nobody in view", [](Inputs& in) {
			 in.bDirectNeutral= false;
			 in.bDirectTorso= false;
		 }},
		{"swapped hand labels", [](Inputs& in) { in.bDirectHandednessSwapped= true; }},
		{"no avatar", [](Inputs& in) { in.bAvatarLoaded= false; }},
		{"osc disabled", [](Inputs& in) { in.bOscEnabled= false; }},
		{"osc socket closed", [](Inputs& in) { in.bOscRunning= false; }},
		{"face and mouth off", [](Inputs& in) { in.bFaceEnabled= in.bAudioEnabled= false; }},
		{"phone off", [](Inputs& in) { in.bFaceEnabled= false; }},
		{"mic off", [](Inputs& in) { in.bAudioEnabled= false; }},
		{"phone port busy", [](Inputs& in) { in.bFaceBound= false; }},
		{"phone waiting, mic off", [](Inputs& in) {
			 in.bFaceStreaming= false;
			 in.bAudioEnabled= false;
		 }},
		{"phone sees no face", [](Inputs& in) { in.bFaceTracked= false; }},
		{"webcam waiting", [](Inputs& in) { in.bWebcamPresent= in.bWebcamTracked= false; }},
		{"webcam sees no face", [](Inputs& in) { in.bWebcamTracked= false; }},
		{"webcam has no face neutral", [](Inputs& in) { in.bWebcamNeutral= false; }},
		{"mic device closed", [](Inputs& in) { in.bAudioDeviceOpen= false; }},
		{"mic model unavailable", [](Inputs& in) { in.bAudioBackendReady= false; }},
		{"mic gated", [](Inputs& in) { in.bAudioGateOpen= false; }},
		{"profile problems", [](Inputs& in) { in.profileProblemCount= 2; }},
		{"custom profile", [](Inputs& in) { in.bProfileAuto= false; }},
	};
}

struct Shape
{
	const char* name;
	bool bDirect;
	int cameraCount;
	bool bImu;
	int bodyCamera;
};

const Shape k_shapes[]= {
	{"tri-camera, body, IMU", false, 3, true, 2},
	{"dual, no body, no IMU", false, 2, false, -1},
	{"single desk camera", false, 1, false, 0},
	{"direct", true, 1, false, 0},
};
} // namespace

static int runPipelineModelTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const std::string& name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO(kTag) << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR(kTag) << "FAIL " << name;
			failures++;
		}
	};

	// (a) Layout: the top row per shape, the side stage, the lanes
	{
		const std::vector<PipelineStageId> multi= PipelineModel::getTopRow(false);
		const std::vector<PipelineStageId> direct= PipelineModel::getTopRow(true);
		check(multi.size() == 7 && multi.front() == id(S::Capture) && multi[1] == id(S::Fusion) &&
				  multi[2] == id(S::Imu) && multi[3] == id(S::BodySolver) && multi[4] == id(S::SourcePose) &&
				  multi[5] == id(S::Retarget) && multi.back() == id(S::Output),
			  "(a) multi-camera top row");
		check(direct.size() == 5 && direct[1] == id(S::DirectSolver) && direct[2] == id(S::SourcePose),
			  "(a) direct top row puts one solver where fusion, IMU and body solver were");
		check(PipelineModel::getSideStage() == id(S::Face), "(a) face is the side stage");

		const auto captureLanes= PipelineModel::getSubStageLanes(id(S::Capture), 3);
		check(captureLanes.size() == 3 && captureLanes[2].size() == 5 && captureLanes[2][0] == id(S::Camera, 2) &&
				  captureLanes[2][4] == id(S::Lift, 2),
			  "(a) capture opens one lane per camera");
		check(PipelineModel::getSubStageLanes(id(S::HandModels, 1), 3).size() == 3,
			  "(a) a sub-stage opens its top-level stage's lanes");
		check(PipelineModel::getSubStageLanes(id(S::DirectSolver), 1).front().size() == 5 &&
				  PipelineModel::getSubStageLanes(id(S::Face), 1).front().size() == 4 &&
				  PipelineModel::getSubStageLanes(id(S::Output), 1).front().size() == 2 &&
				  PipelineModel::getSubStageLanes(id(S::Fusion), 3).empty(),
			  "(a) lanes of the direct solver, face, output, and none for fusion");
		check(PipelineModel::getAllStages(false, 3).size() == 29 && PipelineModel::getAllStages(true, 1).size() == 22,
			  "(a) stage counts: 29 for three cameras, 22 for direct");
		check(PipelineModel::getTopLevel(id(S::Lift, 1)) == id(S::Capture) &&
				  PipelineModel::getTopLevel(id(S::Palm)) == id(S::DirectSolver) &&
				  PipelineModel::getTopLevel(id(S::Fusion)) == id(S::Fusion),
			  "(a) top-level stage of a sub-stage");
	}

	// (b) Text ids round trip, and malformed ones are refused
	{
		bool bRoundTrip= true;
		std::set<std::string> seen;
		for (const bool bDirect : {false, true})
		{
			for (const PipelineStageId& stage : PipelineModel::getAllStages(bDirect, 3))
			{
				const std::string text= PipelineModel::toString(stage);
				PipelineStageId parsed;
				bRoundTrip&= PipelineModel::fromString(text, parsed) && parsed == stage;
				seen.insert(text);
			}
		}
		check(bRoundTrip, "(b) every stage id survives toString and fromString");
		check(seen.count("capture.cam2.hands") == 1 && seen.count("direct.arm") == 1 && seen.count("source") == 1 &&
				  seen.count("face.mic") == 1 && seen.count("body") == 1 && seen.count("capture.cam0.body") == 1,
			  "(b) the ids read as documented");
		PipelineStageId parsed;
		check(!PipelineModel::fromString("", parsed) && !PipelineModel::fromString("nope", parsed) &&
				  !PipelineModel::fromString("capture.cam", parsed) && !PipelineModel::fromString("capture.camx", parsed) &&
				  !PipelineModel::fromString("capture.cam0.nope", parsed) && !PipelineModel::fromString("capture.cam0x", parsed),
			  "(b) malformed ids are refused");
	}

	// (c) A healthy project runs everywhere it has a stage
	{
		const Inputs tri= makeHealthy(false, 3, true, 2);
		bool bAllRunning= true;
		for (const PipelineStageId& stage : PipelineModel::getAllStages(false, 3))
		{
			const eStageState state= stateOf(tri, stage);
			// The overhead cameras run no body model
			const bool bOverheadBody= stage.stage == S::BodyModel && stage.cameraIndex != 2;
			// The webcam face is a direct project's
			const bool bInactive= bOverheadBody || stage.stage == S::Webcam;
			bAllRunning&= bInactive ? state == eStageState::Inactive : state == eStageState::Running;
		}
		check(bAllRunning,
			  "(c) tri-camera: every stage runs, the overhead cameras' body model and the webcam face inactive");

		const Inputs direct= makeHealthy(true, 1, false, 0);
		bool bDirectRunning= true;
		for (const PipelineStageId& stage : PipelineModel::getAllStages(true, 1))
			bDirectRunning&= stateOf(direct, stage) == eStageState::Running;
		check(bDirectRunning, "(c) direct: every stage runs");
		check(reasonOf(direct, id(S::Lift, 0)) == "pipeline.whyDirectNoWorld",
			  "(c) direct: the lift says it stays in camera space");

		const Inputs dual= makeHealthy(false, 2, false, -1);
		check(stateOf(dual, id(S::Imu)) == eStageState::Inactive &&
				  stateOf(dual, id(S::BodySolver)) == eStageState::Inactive &&
				  stateOf(dual, id(S::Fusion)) == eStageState::Running,
			  "(c) dual: IMU and body solver inactive, fusion running");
	}

	// (d) No extrinsics: the fault shows from capture to output, with the
	// reason on the stage that names the fix
	{
		Inputs in= makeHealthy(false, 3, true, 2);
		for (Inputs::Camera& camera : in.cameras)
			camera.bExtrinsics= false;
		in.path= Inputs::ePath::Passthrough;
		check(stateOf(in, id(S::Capture)) == eStageState::Degraded &&
				  stateOf(in, id(S::Lift, 0)) == eStageState::NotConfigured &&
				  stateOf(in, id(S::Fusion)) == eStageState::NotConfigured &&
				  valueOf(in, id(S::Fusion)) == "pipeline.valPassthrough" &&
				  reasonOf(in, id(S::Fusion)) == "pipeline.whyFusionPassthrough",
			  "(d) no extrinsics: capture degraded, lift and fusion not configured, fusion names the passthrough");
		check(stateOf(in, id(S::Imu)) == eStageState::Degraded && stateOf(in, id(S::BodySolver)) == eStageState::Inactive &&
				  stateOf(in, id(S::SourcePose)) == eStageState::Degraded &&
				  stateOf(in, id(S::Retarget)) == eStageState::Inactive &&
				  stateOf(in, id(S::Output)) == eStageState::Degraded &&
				  stateOf(in, id(S::Face)) == eStageState::Running,
			  "(d) no extrinsics: IMU, body solver, source pose, retarget and output follow, face unaffected");
	}

	// (e) A fault names its camera, and a hand out of view is not a fault
	{
		Inputs in= makeHealthy(false, 3, true, 2);
		in.cameras[1].bStreaming= false;
		const StageStatus capture= PipelineModel::computeStatus(in, id(S::Capture));
		check(capture.state == eStageState::NotConfigured && capture.reason.key == "pipeline.whyCameraNotStreamingFmt" &&
				  capture.reason.args[0] == 2.0 && capture.value.args[0] == 2.0 && capture.value.args[1] == 3.0,
			  "(e) a stopped camera: capture names camera 2 and counts 2 of 3 streaming");

		Inputs away= makeHealthy(false, 3, true, 2);
		away.hands[0].bTracked= away.hands[1].bTracked= false;
		check(stateOf(away, id(S::Fusion)) == eStageState::Running && valueOf(away, id(S::Fusion)) == "pipeline.valNoHands",
			  "(e) no hands in view: fusion still runs and says so");
	}

	// (f) The avatar and the direct solver's needs
	{
		Inputs multi= makeHealthy(false, 2, false, -1);
		multi.bAvatarLoaded= false;
		check(stateOf(multi, id(S::Retarget)) == eStageState::NotConfigured &&
				  reasonOf(multi, id(S::Retarget)) == "pipeline.whyRetargetNoAvatar" &&
				  stateOf(multi, id(S::Output)) == eStageState::Running,
			  "(f) multi-camera with no avatar: retarget not configured, output still runs");

		Inputs direct= makeHealthy(true, 1, false, 0);
		direct.bAvatarLoaded= false;
		check(stateOf(direct, id(S::DirectSolver)) == eStageState::NotConfigured &&
				  stateOf(direct, id(S::ArmChain)) == eStageState::NotConfigured &&
				  reasonOf(direct, id(S::Retarget)) == "pipeline.whyDirectNoAvatar",
			  "(f) direct with no avatar: the solver and its sub-stages are not configured");

		Inputs noNeutral= makeHealthy(true, 1, false, 0);
		noNeutral.bDirectNeutral= false;
		check(stateOf(noNeutral, id(S::DirectSolver)) == eStageState::Degraded &&
				  stateOf(noNeutral, id(S::Torso)) == eStageState::Degraded &&
				  stateOf(noNeutral, id(S::ArmChain)) == eStageState::Running,
			  "(f) direct with no neutral: the solver and the torso degraded, the arm chain running");

		Inputs swapped= makeHealthy(true, 1, false, 0);
		swapped.bDirectHandednessSwapped= true;
		check(stateOf(swapped, id(S::DirectSolver)) == eStageState::Degraded &&
				  stateOf(swapped, id(S::Filters)) == eStageState::Degraded &&
				  valueOf(swapped, id(S::Filters)) == "pipeline.valHandednessSwapped" &&
				  reasonOf(swapped, id(S::Filters)) == "pipeline.whyDirectHandednessSwapped" &&
				  stateOf(swapped, id(S::Torso)) == eStageState::Running,
			  "(f) direct with swapped hand labels: the solver and its filters degraded, naming the swap");
	}

	// (f2) A replayed frame from a take with no stage state: the stages that
	// are nothing but that state say so, the rest read from the frame
	{
		Inputs in= makeHealthy(false, 3, true, 2);
		in.bReplay= true;
		in.bStageStateRecorded= false;
		check(valueOf(in, id(S::Imu)) == "pipeline.valNotRecorded" && stateOf(in, id(S::Imu)) == eStageState::Inactive &&
				  valueOf(in, id(S::Output)) == "pipeline.valNotRecorded" &&
				  valueOf(in, id(S::VmcStream)) == "pipeline.valNotRecorded" &&
				  valueOf(in, id(S::Phone)) == "pipeline.valNotRecorded" &&
				  valueOf(in, id(S::Microphone)) == "pipeline.valNotRecorded" &&
				  valueOf(in, id(S::Face)) == "pipeline.valNotRecorded",
			  "(f2) unrecorded replay: IMU, output, phone, microphone and face say not recorded");
		check(stateOf(in, id(S::Capture)) == eStageState::Running && stateOf(in, id(S::Fusion)) == eStageState::Running &&
				  stateOf(in, id(S::DeliveryRules)) == eStageState::Running &&
				  stateOf(in, id(S::Retarget)) == eStageState::Running,
			  "(f2) unrecorded replay: capture, fusion, delivery rules and retarget read from the frame and the config");

		Inputs recorded= makeHealthy(false, 3, true, 2);
		recorded.bReplay= true;
		check(stateOf(recorded, id(S::Imu)) == eStageState::Running &&
				  reasonOf(recorded, id(S::Imu)) == "pipeline.whyImuReplay" &&
				  stateOf(recorded, id(S::Output)) == eStageState::Running,
			  "(f2) recorded replay: the IMU runs with the replay note, the output runs");
	}

	// (g) Every text the rules return is in the string table, a format key
	// holds a format, and every status text in the table is reached
	{
		nlohmann::json table;
		{
			std::ifstream file("resources/localization/en.json");
			if (file.is_open())
				table= nlohmann::json::parse(file, nullptr, false);
		}
		const bool bHaveTable= table.is_object() && table.contains("pipeline") && table["pipeline"].is_object();
		check(bHaveTable, "(g) resources/localization/en.json has a pipeline section");
		if (bHaveTable)
		{
			const nlohmann::json& section= table["pipeline"];
			std::set<std::string> returned;
			auto noteText= [&](const StageText& text) {
				if (!text.isEmpty())
					returned.insert(text.key);
			};
			const std::vector<Fault> faults= makeFaults();
			for (const Shape& shape : k_shapes)
			{
				for (const Fault& fault : faults)
				{
					Inputs in= makeHealthy(shape.bDirect, shape.cameraCount, shape.bImu, shape.bodyCamera);
					fault.apply(in);
					std::vector<PipelineStageId> stages= PipelineModel::getAllStages(shape.bDirect, shape.cameraCount);
					// A stage of a camera the project lacks, as a stale selection asks for
					stages.push_back(id(S::Video, 7));
					for (const PipelineStageId& stage : stages)
					{
						const StageStatus status= PipelineModel::computeStatus(in, stage);
						noteText(status.value);
						noteText(status.reason);
						if (status.value.isEmpty())
						{
							MIKAN_LOG_ERROR(kTag) << "  no value for " << PipelineModel::toString(stage) << " under "
												  << fault.name << " on " << shape.name;
							failures++;
						}
					}
				}
			}

			int missing= 0;
			int badFormat= 0;
			for (const std::string& key : returned)
			{
				const std::string name= key.substr(std::string("pipeline.").size());
				if (!section.contains(name) || !section[name].is_string())
				{
					MIKAN_LOG_ERROR(kTag) << "  not in the table: " << key;
					missing++;
					continue;
				}
				const bool bIsFormat= name.size() > 3 && name.compare(name.size() - 3, 3, "Fmt") == 0;
				const bool bHasSpecifier= section[name].get<std::string>().find('%') != std::string::npos;
				if (bIsFormat != bHasSpecifier)
				{
					MIKAN_LOG_ERROR(kTag) << "  format mismatch: " << key;
					badFormat++;
				}
			}
			check(missing == 0, "(g) every returned text is in the table");
			check(badFormat == 0, "(g) a key ends in Fmt exactly when its text has a specifier");

			int unreached= 0;
			for (auto it= section.begin(); it != section.end(); ++it)
			{
				const std::string& name= it.key();
				const bool bStatusText= name.compare(0, 3, "val") == 0 || name.compare(0, 3, "why") == 0;
				if (bStatusText && returned.count("pipeline." + name) == 0)
				{
					MIKAN_LOG_ERROR(kTag) << "  never returned: pipeline." << name;
					unreached++;
				}
			}
			check(unreached == 0, "(g) every status text in the table is reached by some input");

			int missingNames= 0;
			for (int index= 0; index < (int)S::Count; ++index)
			{
				for (const char* key : {PipelineModel::getNameKey((S)index), PipelineModel::getDescriptionKey((S)index)})
				{
					const std::string name= std::string(key).substr(std::string("pipeline.").size());
					if (!section.contains(name))
					{
						MIKAN_LOG_ERROR(kTag) << "  not in the table: " << key;
						missingNames++;
					}
				}
			}
			for (const eStageState state : {eStageState::Running, eStageState::Degraded, eStageState::Inactive, eStageState::NotConfigured})
			{
				const std::string name= std::string(PipelineModel::getStateKey(state)).substr(std::string("pipeline.").size());
				missingNames+= section.contains(name) ? 0 : 1;
			}
			check(missingNames == 0, "(g) every stage has a name and a description, every state a label");
		}
	}

	MIKAN_LOG_INFO(kTag) << (failures == 0 ? "ALL PASSED" : "FAILED") << " (" << failures << " failures)";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-pipeline-model",
					"Pipeline model: layout per project shape, stage ids, status rules, string table coverage",
					eTestCategory::SelfTest, runPipelineModelTest)
