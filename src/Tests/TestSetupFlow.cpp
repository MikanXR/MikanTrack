#include "TestCommon.h"

#include "SetupFlow.h"

// The guided setup's chain per tracking variant, walked through the pure
// chain function the flow advances by

namespace
{
using eSetup= SetupFlow::eTrackingSetup;
using eStep= SetupFlow::eStep;

// The steps a variant visits from the first prompt, the intrinsics wizard
// once per camera
std::vector<eStep> walkChain(eSetup setup)
{
	std::vector<eStep> chain;
	eStep step= eStep::TrackingSetup;
	while (step != eStep::Inactive && chain.size() < 32)
	{
		chain.push_back(step);
		if (step == eStep::IntrinsicsRunning)
		{
			for (int cameraIndex= 1; cameraIndex < SetupFlow::getRequiredCameraCount(setup); ++cameraIndex)
				chain.push_back(step);
		}
		step= SetupFlow::nextStep(setup, step);
	}
	return chain;
}

const char* stepName(eStep step)
{
	switch (step)
	{
		case eStep::Inactive: return "Inactive";
		case eStep::TrackingSetup: return "TrackingSetup";
		case eStep::AvatarLoad: return "AvatarLoad";
		case eStep::CameraSelection: return "CameraSelection";
		case eStep::CharucoPrint: return "CharucoPrint";
		case eStep::IntrinsicsRunning: return "Intrinsics";
		case eStep::ArucoPrint: return "ArucoPrint";
		case eStep::ExtrinsicsRunning: return "Extrinsics";
		case eStep::HandCalibRunning: return "Hand";
		case eStep::MountingRunning: return "Mounting";
		case eStep::BodyCalibRunning: return "Body";
		case eStep::HandednessCheck: return "HandednessCheck";
		case eStep::NeutralCapture: return "NeutralCapture";
		case eStep::ConfirmCancel: return "ConfirmCancel";
	}
	return "?";
}

std::string chainString(const std::vector<eStep>& chain)
{
	std::string text;
	for (eStep step : chain)
	{
		if (!text.empty())
			text+= " > ";
		text+= stepName(step);
	}
	return text;
}
} // namespace

static int runSetupFlowTest(const TestArgs&)
{
	int failures= 0;
	auto check= [&](bool bCondition, const char* name) {
		if (bCondition)
		{
			MIKAN_LOG_INFO("test-setup-flow") << "PASS " << name;
		}
		else
		{
			MIKAN_LOG_ERROR("test-setup-flow") << "FAIL " << name;
			failures++;
		}
	};
	auto checkChain= [&](eSetup setup, const std::vector<eStep>& expected, const char* name) {
		const std::vector<eStep> chain= walkChain(setup);
		if (chain != expected)
			MIKAN_LOG_ERROR("test-setup-flow") << "  got " << chainString(chain);
		check(chain == expected, name);
	};

	const std::vector<eStep> dual= {eStep::TrackingSetup, eStep::CameraSelection, eStep::CharucoPrint,
									eStep::IntrinsicsRunning, eStep::IntrinsicsRunning, eStep::ArucoPrint,
									eStep::ExtrinsicsRunning, eStep::HandCalibRunning};
	std::vector<eStep> joyCons= dual;
	joyCons.push_back(eStep::MountingRunning);
	const std::vector<eStep> tri= {eStep::TrackingSetup, eStep::CameraSelection, eStep::CharucoPrint,
								   eStep::IntrinsicsRunning, eStep::IntrinsicsRunning, eStep::IntrinsicsRunning,
								   eStep::ArucoPrint, eStep::ExtrinsicsRunning, eStep::HandCalibRunning,
								   eStep::BodyCalibRunning};
	const std::vector<eStep> singleFront= {eStep::TrackingSetup, eStep::AvatarLoad, eStep::CameraSelection,
										   eStep::CharucoPrint, eStep::IntrinsicsRunning, eStep::HandednessCheck,
										   eStep::NeutralCapture};
	const std::vector<eStep> singleDesk= {eStep::TrackingSetup, eStep::AvatarLoad, eStep::CameraSelection,
										  eStep::CharucoPrint, eStep::IntrinsicsRunning, eStep::ArucoPrint,
										  eStep::ExtrinsicsRunning, eStep::HandCalibRunning, eStep::BodyCalibRunning};

	checkChain(eSetup::DualOverhead, dual, "(a) the dual overhead chain ends at the hand wizard");
	checkChain(eSetup::DualOverheadJoyCons, joyCons, "(a) the Joy-Con chain adds the mounting wizard");
	checkChain(eSetup::TriCameraFront, tri, "(a) the tri-camera chain runs intrinsics three times and ends at the body wizard");
	checkChain(eSetup::SingleFront, singleFront,
			   "(b) the single-front chain loads the avatar, calibrates one camera's intrinsics, checks the hands' "
			   "left and right and captures the neutral");
	checkChain(eSetup::SingleDesk, singleDesk,
			   "(b) the single-desk chain loads the avatar and runs the marker, hand and body wizards on one camera");

	// Every chain finishes from its last step, and the steps that end a
	// chain never continue for any variant
	const eSetup setups[]= {eSetup::DualOverhead, eSetup::DualOverheadJoyCons, eSetup::TriCameraFront,
							eSetup::SingleFront, eSetup::SingleDesk};
	bool bFinal= true;
	for (eSetup setup : setups)
	{
		bFinal&= SetupFlow::nextStep(setup, eStep::MountingRunning) == eStep::Inactive;
		bFinal&= SetupFlow::nextStep(setup, eStep::BodyCalibRunning) == eStep::Inactive;
		bFinal&= SetupFlow::nextStep(setup, eStep::NeutralCapture) == eStep::Inactive;
		bFinal&= SetupFlow::nextStep(setup, eStep::ConfirmCancel) == eStep::Inactive;
		bFinal&= SetupFlow::nextStep(setup, eStep::Inactive) == eStep::Inactive;
	}
	check(bFinal, "(c) the mounting, body and neutral steps finish every variant");

	bool bHandednessSingleFrontOnly= true;
	for (eSetup setup : setups)
	{
		const bool bReaches= SetupFlow::nextStep(setup, eStep::IntrinsicsRunning) == eStep::HandednessCheck;
		bHandednessSingleFrontOnly&= bReaches == (setup == eSetup::SingleFront);
	}
	check(bHandednessSingleFrontOnly, "(d) only the single-front variant reaches the handedness check");

	// The lens step's exits: a calibration runs the intrinsics wizard next,
	// and a table or skipped lens goes straight to what follows it
	using eLens= SetupFlow::eLensChoice;
	auto stepAfterLens= [](eLens choice) {
		return SetupFlow::nextStep(eSetup::SingleFront, SetupFlow::stepCompletedByLensChoice(choice));
	};
	check(stepAfterLens(eLens::Phone) == eStep::IntrinsicsRunning &&
			  stepAfterLens(eLens::Printed) == eStep::IntrinsicsRunning,
		  "(e) a phone or printed calibration goes on to the intrinsics wizard");
	check(stepAfterLens(eLens::Table) == eStep::HandednessCheck && stepAfterLens(eLens::Skip) == eStep::HandednessCheck,
		  "(e) a table or skipped lens passes over the intrinsics wizard");

	MIKAN_LOG_INFO("test-setup-flow") << (failures == 0 ? "ALL PASS" : "FAILURES") << " (" << failures << ")";
	return failures == 0 ? 0 : 1;
}

MIKAN_REGISTER_TEST("--test-setup-flow", "Guided setup: the step chain of every tracking variant",
					eTestCategory::SelfTest, runSetupFlowTest);
