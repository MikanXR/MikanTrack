#include "TestCommon.h"

#include <array>
#include <chrono>
#include <sstream>
#include <thread>

#include "AudioService.h"
#include "SteadyClock.h"
#include "VisemeFit.h"

// The microphone mouth stage live, without the UI: lists the capture
// devices, runs the service on one for a few seconds, and reports the level,
// the gate, the backend's run time, and the mouth frames it published, so a
// microphone problem is told from a model problem before the panel is
// involved.
//
//   --audio-probe [seconds] [--device name] [--gate dbfs] [--gain db]

static int runAudioProbe(const TestArgs& args)
{
	double seconds= 5.0;
	AudioServiceConfig config;
	config.enabled= true;
	for (size_t index= 0; index < args.size(); ++index)
	{
		const std::string& arg= args[index];
		const bool bHasValue= index + 1 < args.size();
		if (arg == "--device" && bHasValue)
			config.deviceName= args[++index];
		else if (arg == "--gate" && bHasValue)
			config.gateDbfs= std::stof(args[++index]);
		else if (arg == "--gain" && bHasValue)
			config.gainDb= std::stof(args[++index]);
		else if (arg.rfind("--", 0) != 0)
			seconds= std::stod(arg);
	}

	AudioService service;
	if (!service.startup())
	{
		MIKAN_LOG_ERROR("audio-probe") << "the audio service failed to start";
		return 1;
	}

	const std::vector<std::string> devices= service.listCaptureDevices();
	MIKAN_LOG_INFO("audio-probe") << devices.size() << " capture devices:";
	for (const std::string& name : devices)
		MIKAN_LOG_INFO("audio-probe") << "  " << name;

	service.setConfig(config);

	// Poll at the UI's pace and count what arrives
	const double startMs= steadyNowMs();
	uint64_t lastSequence= 0;
	uint64_t frameCount= 0;
	float peakDbfs= -100.f;
	std::array<int, VISEME_COUNT> visemeFrames{};
	int openHops= 0;
	int polls= 0;
	double lastReportMs= startMs;
	while (steadyNowMs() - startMs < seconds * 1000.0)
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(10));
		const AudioServiceStatus status= service.getStatus();
		++polls;
		peakDbfs= std::max(peakDbfs, status.levelDbfs);
		if (status.gateOpen)
			++openHops;
		if (status.latest.valid && status.latest.sequence != lastSequence)
		{
			// Every sequence step is a frame, seen or not
			frameCount+= status.latest.sequence - lastSequence;
			lastSequence= status.latest.sequence;
			visemeFrames[dominantViseme(status.latest.frame.weights)]++;
		}

		const double nowMs= steadyNowMs();
		if (nowMs - lastReportMs >= 1000.0)
		{
			lastReportMs= nowMs;
			MIKAN_LOG_INFO("audio-probe") << "level " << status.levelDbfs << " dBFS, gate " << (status.gateOpen ? "open" : "closed")
										  << ", frames " << frameCount << ", viseme "
										  << (status.latest.valid ? visemeName(dominantViseme(status.latest.frame.weights)) : "none")
										  << ", infer " << status.inferMsEma << " ms";
		}
	}

	// The wizard's capture tap: a second of the raw stream through the fit's
	// front end, the data path a prompt takes
	std::vector<float> captured;
	if (service.startCapture(2.f))
	{
		std::this_thread::sleep_for(std::chrono::milliseconds(1000));
		service.stopCapture(captured);
	}
	std::vector<MfccVector> speechVectors;
	extractSpeechVectors(captured.data(), captured.size(), VadGateConfig(), speechVectors);
	MIKAN_LOG_INFO("audio-probe") << "capture tap: " << captured.size() << " samples in 1 s, " << speechVectors.size()
								  << " hops passed the energy gate, suggested gate "
								  << suggestGateDbfs(captured.data(), captured.size(), config.gainDb) << " dBFS";

	const AudioServiceStatus status= service.getStatus();
	MIKAN_LOG_INFO("audio-probe") << "device " << (status.deviceOpen ? "open" : "NOT open") << ": " << status.deviceName
								  << " at " << status.deviceSampleRate << " Hz";
	MIKAN_LOG_INFO("audio-probe") << "model " << (status.backendReady ? "ready" : "NOT ready")
								  << (status.backendReady ? "" : ": " + status.backendReason);
	std::ostringstream seen;
	for (int viseme= 0; viseme < VISEME_COUNT; ++viseme)
	{
		if (visemeFrames[viseme] > 0)
			seen << " " << visemeName(viseme) << " " << visemeFrames[viseme];
	}
	MIKAN_LOG_INFO("audio-probe") << frameCount << " frames in " << seconds << " s (" << (double)frameCount / seconds
								  << " per second), peak level " << peakDbfs << " dBFS, gate open on " << openHops << " of "
								  << polls << " polls, dropped blocks " << status.droppedBlocks << ", dominant viseme per polled frame:"
								  << seen.str();

	service.shutdown();
	const bool bOk= status.deviceOpen && status.backendReady && frameCount > 0 && captured.size() >= AUDIO_SAMPLE_RATE / 2;
	MIKAN_LOG_INFO("audio-probe") << (bOk ? "PROBE PASS" : "PROBE FAIL");
	return bOk ? 0 : 1;
}

MIKAN_REGISTER_TEST("--audio-probe", "Live microphone mouth stage: devices, level, gate, model, frames per second",
					eTestCategory::Hardware, runAudioProbe)
