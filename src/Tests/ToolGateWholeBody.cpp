#include "TestCommon.h"

#include <cfloat>
#include <iomanip>

#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "BodyHandHints.h"
#include "DirectRetarget.h"
#include "DirectStageSolver.h"
#include "FaceHead.h"
#include "FaceRecorder.h"
#include "FaceTypes.h"
#include "FrameRecorder.h"
#include "HandSideMatch.h"
#include "OnnxSession.h"
#include "RtmPoseModel.h"
#include "SyntheticAvatar.h"
#include "WholeBodyFaceRatios.h"

// Whole-body landmark gate: re-runs the whole-body model over a recording's
// raw frames from one camera and measures what its face and hand blocks are
// worth from that camera's position, against the truth the same take carries.
//
//   --gate-wholebody <recording.jsonl> [cameraIndex] [rest=<s>-<s>] [frames.csv]
//
// cameraIndex defaults to the direct camera, else the recording's body-pose
// camera. The raw frames beside the recording are required (raw frame
// recording is opt-in). rest names the seconds of the take, from its first
// frame, where the face is held still (default 0-3).
//
// The model runs twice over the same frames: on every frame, and on the
// frames the camera's live poseFrameDivider leaves, with the tracker's own
// box tracking at that cadence. The live stage runs at the divider, so the
// divided pass is what it would have measured, and the every-frame pass is
// what a faster cadence would buy.
//
// Face, from the face block alone:
//   jitter   each point's scatter about its mean over the rest window, with
//            the face's centroid removed per frame, px and face widths, by
//            contour
// Face shapes, with a face sidecar (the phone streaming during the take):
//   the shape ratios against the ARKit columns they would stand in for, as a
//   correlation at the lag that maximizes it (the two streams' latencies
//   differ). Each sided shape is correlated against the phone's column of the
//   same side and of the other, so the take says which pairing is the same
//   eye rather than the convention being assumed.
//   blinks   the eye openness against the phone's blink column: the open and
//            closed levels and their separation, then the phone's blinks as
//            events and how many the openness finds at the threshold that
//            scores best, with the false ones, at both cadences
//   models   the same blink numbers for each face model found in models/
//            (RTMPose face6, MediaPipe Face Landmarker), run on a crop the
//            face block places, with its time
// Hands, on a camera with extrinsics (the stereo rig's triangulated hands as
// truth, projected into the camera):
//   position  the whole-body hand's wrist and knuckles against the fused
//             hand's, and whether the whole-body hand's box holds the fused
//             palm, the question a search hint asks
//   overlap   the whole-body hand's box against the hand slot's own
//   side      which side each hand slot is, by the hand's shape against the
//             two whole-body hands and by the direct solver's wrist test,
//             each against the fused side
//
// Everything is reported and nothing is gated: these numbers set the gates.

namespace
{
constexpr float kScoreGate= 0.3f;
// A phone sample older than this is not the face at the frame
constexpr double kPhoneStaleMs= 200.0;
// The phone's blink column as events
constexpr float kPhoneBlinkOnLevel= 0.6f;
constexpr float kPhoneBlinkOffLevel= 0.3f;
constexpr float kPhoneOpenLevel= 0.1f;
// A lesser closure of the phone's column, under its blink level
constexpr float kPhonePartialOnLevel= 0.35f;
constexpr float kPhonePartialOffLevel= 0.25f;
// The phone has one eye shut and the other open with its blink columns this
// far apart, and a signal is judged on this many such frames or more
constexpr float kPhoneOneEyeGap= 0.4f;
constexpr int kMinOneEyeFrames= 8;
// A blink shorter than this is a fast one
constexpr double kFastBlinkMs= 150.0;
// How far outside a phone blink the camera's may start or end and still be it
constexpr double kBlinkMatchSlackMs= 100.0;
// The direct solver's wrist agreement distance
constexpr float kHandAgreementShoulderWidths= 0.5f;
// The shape match's side needs the better hand this much closer than the other
constexpr float kShapeMarginRatio= 0.8f;
// The fused hands count as close together within this many shoulder widths
constexpr float kHandsCloseShoulderWidths= 1.0f;

struct Stats
{
	std::vector<float> values;
	void add(float value) { values.push_back(value); }
	int count() const { return (int)values.size(); }
	float percentile(float fraction) const
	{
		if (values.empty())
			return 0.f;
		std::vector<float> sorted= values;
		std::sort(sorted.begin(), sorted.end());
		return sorted[std::min(sorted.size() - 1, (size_t)((float)sorted.size() * fraction))];
	}
	float p50() const { return percentile(0.5f); }
	float p90() const { return percentile(0.9f); }
	float mean() const
	{
		if (values.empty())
			return 0.f;
		double sum= 0.0;
		for (float value : values)
			sum+= value;
		return (float)(sum / (double)values.size());
	}
	float variance() const
	{
		if (values.size() < 2)
			return 0.f;
		const float average= mean();
		double sum= 0.0;
		for (float value : values)
			sum+= (double)(value - average) * (double)(value - average);
		return (float)(sum / (double)(values.size() - 1));
	}
};

int percent(int part, int whole)
{
	return whole > 0 ? (100 * part + whole / 2) / whole : 0;
}

// One model result of a pass
struct ModelSample
{
	int replayFrameIndex= -1;
	double timestampMs= 0.0;
	int frameWidth= 0;
	int frameHeight= 0;
	BodyPoseObservation body;
	WholeBodyFaceRatios ratios;
	// The camera's own recorded hand slots on the frame
	std::array<RecordedHandInput, 2> slots;
};

// One stored frame of the camera with the observation the body stage emitted
// on it, a held one on a divided-out frame
struct FrameObservation
{
	int replayFrameIndex= -1;
	int64_t frameIndex= -1;
	BodyPoseObservation body;
	double timestampMs= 0.0;
	// The model ran on this frame
	bool bModelFrame= false;
};

// outAllFrames, when given, receives every stored frame in order with the
// stage's observation on it, held ones included
bool runModelPass(const char* tag, const TrackingReplay& replay, const std::string& frameDirectory, int cameraIndex,
				  int frameDivider, std::vector<ModelSample>& outSamples, Stats& outInferenceMs, int& outFramesMissing,
				  std::vector<FrameObservation>* outAllFrames= nullptr)
{
	const CameraProfile& profile= replay.getRecordedConfig().camera(cameraIndex);
	BodyPoseTrackerConfig trackerConfig;
	trackerConfig.backend= eBodyPoseBackend::Rtmw3dWholeBody;
	trackerConfig.frameDivider= frameDivider;
	trackerConfig.detectorIntervalFrames= profile.bodyPose.detectorIntervalFrames;
	BodyPoseTracker tracker;
	if (!tracker.load("models", "directml", trackerConfig))
	{
		MIKAN_LOG_ERROR(tag) << "Failed to load the whole-body model (models/rtmw3d_wholebody.onnx)";
		return false;
	}

	outSamples.clear();
	outFramesMissing= 0;
	int64_t lastModelFrameIndex= -1;
	for (int frameIndex= 0; frameIndex < replay.getFrameCount(); ++frameIndex)
	{
		const RecordedFrame& recorded= *replay.getFrame(frameIndex).recorded;
		for (const RecordedCameraInput& fresh : recorded.freshCameras)
		{
			if (fresh.cameraIndex != cameraIndex)
				continue;
			const cv::Mat image= cv::imread(
				FrameRecorder::makeFramePath(frameDirectory, cameraIndex, fresh.frameIndex), cv::IMREAD_COLOR);
			if (image.empty())
			{
				outFramesMissing++;
				continue;
			}

			BodyPoseObservation observation;
			const auto start= std::chrono::steady_clock::now();
			tracker.process(image, observation);
			const float elapsedMs=
				(float)std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
			if (outAllFrames != nullptr)
			{
				outAllFrames->push_back({frameIndex, fresh.frameIndex, observation, fresh.timestampMs,
											observation.valid && observation.modelFrameIndex != lastModelFrameIndex});
			}
			// A divided-out frame re-emits the held result
			if (!observation.valid || observation.modelFrameIndex == lastModelFrameIndex)
				continue;
			lastModelFrameIndex= observation.modelFrameIndex;
			outInferenceMs.add(elapsedMs);

			ModelSample sample;
			sample.replayFrameIndex= frameIndex;
			sample.timestampMs= fresh.timestampMs;
			sample.frameWidth= image.cols;
			sample.frameHeight= image.rows;
			sample.body= observation;
			sample.ratios= computeWholeBodyFaceRatios(observation.wholeBody);
			sample.slots= fresh.hands;
			outSamples.push_back(sample);
		}
	}
	return true;
}

// -- Face jitter -----------------------------------------------------------

void reportFaceJitter(const char* tag, const std::vector<ModelSample>& samples, double restStartMs, double restEndMs)
{
	static const char* kContourNames[WHOLEBODY_FACE_CONTOUR_COUNT]= {
		"jaw", "right brow", "left brow", "nose bridge", "nose base", "right eye", "left eye", "outer lip", "inner lip"};

	// Each point about the face's own centroid on its frame, so a head that
	// drifts during the hold does not read as landmark jitter
	using FacePoints= std::array<glm::vec2, WHOLEBODY_FACE_LANDMARK_COUNT>;
	FacePoints sums{};
	std::array<Stats, WHOLEBODY_FACE_LANDMARK_COUNT> scores;
	std::vector<FacePoints> window;
	Stats faceWidth;
	for (const ModelSample& sample : samples)
	{
		if (!sample.ratios.valid || sample.timestampMs < restStartMs || sample.timestampMs > restEndMs)
			continue;
		glm::vec2 centroid(0.f);
		for (const glm::vec3& point : sample.body.wholeBody.face)
			centroid+= glm::vec2(point);
		centroid/= (float)WHOLEBODY_FACE_LANDMARK_COUNT;

		FacePoints centered;
		faceWidth.add(sample.ratios.faceWidthPx);
		for (int index= 0; index < WHOLEBODY_FACE_LANDMARK_COUNT; ++index)
		{
			centered[index]= glm::vec2(sample.body.wholeBody.face[index]) - centroid;
			sums[index]+= centered[index];
			scores[index].add(sample.body.wholeBody.faceScores[index]);
		}
		window.push_back(centered);
	}
	if (window.size() < 5)
	{
		MIKAN_LOG_INFO(tag) << "face jitter: fewer than 5 model frames in the rest window, not measured";
		return;
	}

	const float width= faceWidth.p50();
	MIKAN_LOG_INFO(tag) << "face jitter over the rest window (" << window.size() << " model frames, face width "
						<< width << " px): scatter about the mean with the face's centroid removed, px (face widths), and score";
	for (int contourIndex= 0; contourIndex < WHOLEBODY_FACE_CONTOUR_COUNT; ++contourIndex)
	{
		const WholeBodyFaceContour& contour= WHOLEBODY_FACE_CONTOURS[contourIndex];
		Stats scatter;
		Stats score;
		for (int index= contour.first; index <= contour.last; ++index)
		{
			const glm::vec2 mean= sums[index] / (float)window.size();
			double squared= 0.0;
			for (const FacePoints& centered : window)
			{
				const glm::vec2 delta= centered[index] - mean;
				squared+= (double)glm::dot(delta, delta);
			}
			scatter.add((float)std::sqrt(squared / (double)window.size()));
			score.add(scores[index].p50());
		}
		MIKAN_LOG_INFO(tag) << "  " << kContourNames[contourIndex] << ": p50 " << scatter.p50() << " ("
							<< scatter.p50() / width << "), worst " << scatter.percentile(1.f) << " ("
							<< scatter.percentile(1.f) / width << "), score p50 " << score.p50();
	}
}

// -- Face shapes against the phone -----------------------------------------

// The phone's column at a time, false when the phone has no tracked face there
bool phoneValueAt(const std::vector<RecordedFaceSample>& faceSamples, double timestampMs, int column, float& outValue)
{
	const RecordedFaceSample* sample= FaceSidecar::findAtOrBefore(faceSamples, timestampMs);
	if (sample == nullptr || !sample->bTracked || timestampMs - sample->timestampMs > kPhoneStaleMs)
		return false;
	outValue= sample->blendshapes[(size_t)column];
	return true;
}

using ShapeGetter= float (*)(const WholeBodyFaceRatios&);

struct Correlation
{
	float r= 0.f;
	double lagMs= 0.0;
	int count= 0;
};

float pearson(const std::vector<float>& a, const std::vector<float>& b)
{
	if (a.size() < 3)
		return 0.f;
	double meanA= 0.0, meanB= 0.0;
	for (size_t i= 0; i < a.size(); ++i)
	{
		meanA+= a[i];
		meanB+= b[i];
	}
	meanA/= (double)a.size();
	meanB/= (double)a.size();
	double covariance= 0.0, varianceA= 0.0, varianceB= 0.0;
	for (size_t i= 0; i < a.size(); ++i)
	{
		covariance+= (a[i] - meanA) * (b[i] - meanB);
		varianceA+= (a[i] - meanA) * (a[i] - meanA);
		varianceB+= (b[i] - meanB) * (b[i] - meanB);
	}
	const double denominator= std::sqrt(varianceA * varianceB);
	return denominator > 1e-12 ? (float)(covariance / denominator) : 0.f;
}

// One reading of the two eyes, side 0 the person's left, higher the more open
struct EyeSample
{
	double timestampMs= 0.0;
	bool bValid= false;
	// The body model ran on this frame
	bool bModelFrame= false;
	float openness[2]= {0.f, 0.f};
};
using EyeSeries= std::vector<EyeSample>;

// Values at their times against the phone's column, at the lag (the phone's
// clock ahead of the camera's, ms) where the correlation is strongest
Correlation correlateValues(const std::vector<double>& timesMs, const std::vector<float>& values,
							const std::vector<RecordedFaceSample>& faceSamples, int column)
{
	Correlation best;
	std::vector<float> shape;
	std::vector<float> phone;
	for (double lagMs= -300.0; lagMs <= 300.0; lagMs+= 20.0)
	{
		shape.clear();
		phone.clear();
		for (size_t index= 0; index < timesMs.size(); ++index)
		{
			float value= 0.f;
			if (!phoneValueAt(faceSamples, timesMs[index] + lagMs, column, value))
				continue;
			shape.push_back(values[index]);
			phone.push_back(value);
		}
		const float r= pearson(shape, phone);
		// A column the phone never moved correlates with nothing at any lag,
		// and still reports the frames it was read on
		if (best.count == 0 || std::fabs(r) > std::fabs(best.r))
		{
			best.r= r;
			best.lagMs= lagMs;
			best.count= (int)shape.size();
		}
	}
	return best;
}

Correlation correlate(const std::vector<ModelSample>& samples, const std::vector<RecordedFaceSample>& faceSamples,
					  ShapeGetter getter, int column)
{
	std::vector<double> timesMs;
	std::vector<float> values;
	for (const ModelSample& sample : samples)
	{
		if (!sample.ratios.valid)
			continue;
		timesMs.push_back(sample.timestampMs);
		values.push_back(getter(sample.ratios));
	}
	return correlateValues(timesMs, values, faceSamples, column);
}

Correlation correlateEye(const EyeSeries& series, int sideIndex, const std::vector<RecordedFaceSample>& faceSamples,
						 int column)
{
	std::vector<double> timesMs;
	std::vector<float> values;
	for (const EyeSample& sample : series)
	{
		if (!sample.bValid)
			continue;
		timesMs.push_back(sample.timestampMs);
		values.push_back(sample.openness[sideIndex]);
	}
	return correlateValues(timesMs, values, faceSamples, column);
}

void logCorrelation(const char* tag, const char* shapeName, const char* columnName, const Correlation& correlation)
{
	MIKAN_LOG_INFO(tag) << "  " << shapeName << " vs " << columnName << ": r " << correlation.r << " at lag "
						<< correlation.lagMs << " ms (" << correlation.count << " frames)";
}

struct BlinkEvent
{
	double startMs= 0.0;
	double endMs= 0.0;
};

// The phone's blink column as events, in the phone's own time
std::vector<BlinkEvent> phoneBlinkEvents(const std::vector<RecordedFaceSample>& faceSamples, int column,
										 double firstMs, double lastMs, float onLevel, float offLevel)
{
	std::vector<BlinkEvent> events;
	bool bClosed= false;
	BlinkEvent current;
	for (const RecordedFaceSample& sample : faceSamples)
	{
		if (sample.timestampMs < firstMs || sample.timestampMs > lastMs)
			continue;
		const float value= sample.bTracked ? sample.blendshapes[(size_t)column] : 0.f;
		if (!bClosed && value >= onLevel)
		{
			bClosed= true;
			current.startMs= sample.timestampMs;
		}
		else if (bClosed && value <= offLevel)
		{
			bClosed= false;
			current.endMs= sample.timestampMs;
			events.push_back(current);
		}
	}
	return events;
}

void reportBlinks(const char* tag, const char* passName, const char* eyeName, const EyeSeries& samples,
				  const std::vector<RecordedFaceSample>& faceSamples, int sideIndex, int column, double lagMs)
{
	// The openness while the phone reads the eye open and closed
	Stats open;
	Stats closed;
	for (const EyeSample& sample : samples)
	{
		float value= 0.f;
		if (!sample.bValid || !phoneValueAt(faceSamples, sample.timestampMs + lagMs, column, value))
			continue;
		if (value <= kPhoneOpenLevel)
			open.add(sample.openness[sideIndex]);
		else if (value >= kPhoneBlinkOnLevel)
			closed.add(sample.openness[sideIndex]);
	}
	if (open.count() < 10 || samples.empty())
	{
		MIKAN_LOG_INFO(tag) << "  " << passName << " " << eyeName << ": too few open-eye frames to measure";
		return;
	}
	const float openLevel= open.p50();
	const float pooledDeviation= std::sqrt(0.5f * (open.variance() + closed.variance()));
	MIKAN_LOG_INFO(tag) << "  " << passName << " " << eyeName << ": openness open p50 " << openLevel << " ("
						<< open.count() << " frames), closed p50 " << closed.p50() << " (" << closed.count()
						<< " frames), separation "
						<< (closed.count() >= 3 && pooledDeviation > 1e-6f
								? (open.mean() - closed.mean()) / pooledDeviation
								: 0.f)
						<< " deviations";

	// The phone's blinks, carried into the camera's time
	const double firstMs= samples.front().timestampMs + lagMs;
	const double lastMs= samples.back().timestampMs + lagMs;
	std::vector<BlinkEvent> truth=
		phoneBlinkEvents(faceSamples, column, firstMs, lastMs, kPhoneBlinkOnLevel, kPhoneBlinkOffLevel);
	// The phone's lesser closures too. A camera closure on one is neither a
	// found blink nor a false one: the phone's column peaks under its blink
	// level on closures the camera sees as full.
	std::vector<BlinkEvent> partial=
		phoneBlinkEvents(faceSamples, column, firstMs, lastMs, kPhonePartialOnLevel, kPhonePartialOffLevel);
	for (std::vector<BlinkEvent>* events : {&truth, &partial})
	{
		for (BlinkEvent& event : *events)
		{
			event.startMs-= lagMs;
			event.endMs-= lagMs;
		}
	}
	if (truth.empty())
	{
		MIKAN_LOG_INFO(tag) << "    the phone saw no blink of this eye";
		return;
	}
	int fastCount= 0;
	for (const BlinkEvent& event : truth)
		fastCount+= (event.endMs - event.startMs) < kFastBlinkMs ? 1 : 0;

	// The threshold, as a fraction of the open level, that scores best
	struct Score
	{
		float threshold= 0.f;
		int detected= 0;
		int detectedFast= 0;
		int partialEvents= 0;
		int falseEvents= 0;
		// False events seen on one frame alone
		int falseOneFrame= 0;
		float f1= -1.f;
		Stats onsetDelayMs;
	};
	Score best;
	for (float threshold= 0.5f; threshold <= 0.901f; threshold+= 0.05f)
	{
		std::vector<BlinkEvent> found;
		bool bClosed= false;
		BlinkEvent current;
		for (const EyeSample& sample : samples)
		{
			const bool bBelow= sample.bValid && sample.openness[sideIndex] < threshold * openLevel;
			if (bBelow && !bClosed)
				current.startMs= sample.timestampMs;
			if (bBelow)
				current.endMs= sample.timestampMs;
			if (!bBelow && bClosed)
				found.push_back(current);
			bClosed= bBelow;
		}
		if (bClosed)
			found.push_back(current);

		auto overlaps= [](const BlinkEvent& phone, const BlinkEvent& camera) {
			return camera.endMs >= phone.startMs - kBlinkMatchSlackMs &&
				camera.startMs <= phone.endMs + kBlinkMatchSlackMs;
		};
		Score score;
		score.threshold= threshold;
		for (const BlinkEvent& phone : truth)
		{
			const BlinkEvent* match= nullptr;
			for (const BlinkEvent& camera : found)
			{
				if (match == nullptr && overlaps(phone, camera))
					match= &camera;
			}
			if (match == nullptr)
				continue;
			score.detected++;
			score.detectedFast+= (phone.endMs - phone.startMs) < kFastBlinkMs ? 1 : 0;
			// Against the phone's own clock, so the streams' latency is in it
			score.onsetDelayMs.add((float)(match->startMs - (phone.startMs + lagMs)));
		}
		for (const BlinkEvent& camera : found)
		{
			bool bMatched= false;
			for (const BlinkEvent& phone : truth)
				bMatched= bMatched || overlaps(phone, camera);
			if (bMatched)
				continue;
			bool bPartial= false;
			for (const BlinkEvent& phone : partial)
				bPartial= bPartial || overlaps(phone, camera);
			if (bPartial)
			{
				score.partialEvents++;
				continue;
			}
			score.falseEvents++;
			score.falseOneFrame+= camera.endMs == camera.startMs ? 1 : 0;
		}
		const int missed= (int)truth.size() - score.detected;
		score.f1= 2.f * (float)score.detected / (float)(2 * score.detected + missed + score.falseEvents);
		if (score.f1 > best.f1)
			best= score;
	}

	const double minutes= (samples.back().timestampMs - samples.front().timestampMs) / 60000.0;
	MIKAN_LOG_INFO(tag) << "    blinks at " << best.threshold << " of the open level: found " << best.detected << " of "
						<< truth.size() << " (" << percent(best.detected, (int)truth.size()) << "%), fast ones "
						<< best.detectedFast << " of " << fastCount << ", on a lesser phone closure "
						<< best.partialEvents << ", false " << best.falseEvents << " ("
						<< (minutes > 0.0 ? (float)(best.falseEvents / minutes) : 0.f) << " a minute, "
						<< best.falseOneFrame << " of one frame), onset after the phone's p50 "
						<< best.onsetDelayMs.p50() << " ms";
}

struct NamedEyeSeries
{
	const char* passName;
	const EyeSeries* series;
};

// One eye signal over its passes: each eye against the phone blink column it
// follows best on the first pass, at that lag, with the other column's
// correlation beside it, since a signal that follows both alike cannot tell
// a wink's eye
void reportBlinkSignal(const char* tag, const std::vector<NamedEyeSeries>& passes,
					   const std::vector<RecordedFaceSample>& faceSamples)
{
	static const char* kEyeNames[2]= {"left eye", "right eye"};
	const int blinkColumns[2]= {arkitBlendshapeFromName("eyeBlinkLeft"), arkitBlendshapeFromName("eyeBlinkRight")};

	// The frames where the phone has one eye shut and the other open say
	// which camera eye is which phone column, and whether the signal tells
	// the eyes apart at all. A blink of both eyes cannot.
	int oneEyeFrames= 0;
	int sameSideLower= 0;
	for (const EyeSample& sample : *passes.front().series)
	{
		float phoneLeft= 0.f;
		float phoneRight= 0.f;
		if (!sample.bValid || !phoneValueAt(faceSamples, sample.timestampMs, blinkColumns[0], phoneLeft) ||
			!phoneValueAt(faceSamples, sample.timestampMs, blinkColumns[1], phoneRight) ||
			std::fabs(phoneLeft - phoneRight) < kPhoneOneEyeGap)
			continue;
		oneEyeFrames++;
		const int phoneClosedSide= phoneLeft > phoneRight ? 0 : 1;
		sameSideLower+= sample.openness[phoneClosedSide] < sample.openness[1 - phoneClosedSide] ? 1 : 0;
	}
	const bool bSame= oneEyeFrames >= kMinOneEyeFrames && sameSideLower * 4 >= oneEyeFrames * 3;
	const bool bCrossed= oneEyeFrames >= kMinOneEyeFrames && sameSideLower * 4 <= oneEyeFrames;
	MIKAN_LOG_INFO(tag) << "  one-eye frames (the phone's blink columns " << kPhoneOneEyeGap << " apart): " << oneEyeFrames
						<< ", the camera eye on the phone's closed side reads the lower on "
						<< percent(sameSideLower, oneEyeFrames) << "%: "
						<< (bSame ? "the signal tells the eyes apart, same sides as the phone"
								  : bCrossed ? "the signal tells the eyes apart, sides crossed against the phone"
											 : "the eyes are not told apart, each paired by correlation");

	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		Correlation chosen;
		Correlation other;
		int chosenColumn= blinkColumns[sideIndex];
		if (bSame || bCrossed)
		{
			chosenColumn= blinkColumns[bSame ? sideIndex : 1 - sideIndex];
			chosen= correlateEye(*passes.front().series, sideIndex, faceSamples, chosenColumn);
			other= correlateEye(*passes.front().series, sideIndex, faceSamples,
								blinkColumns[bSame ? 1 - sideIndex : sideIndex]);
		}
		else
		{
			for (int candidate : blinkColumns)
			{
				const Correlation correlation= correlateEye(*passes.front().series, sideIndex, faceSamples, candidate);
				if (-correlation.r > -chosen.r)
				{
					other= chosen;
					chosen= correlation;
					chosenColumn= candidate;
				}
				else
				{
					other= correlation;
				}
			}
		}
		MIKAN_LOG_INFO(tag) << "  " << kEyeNames[sideIndex] << " against the phone's " << arkitBlendshapeName(chosenColumn)
							<< " (r " << chosen.r << ", lag " << chosen.lagMs << " ms), the other column r " << other.r;
		for (const NamedEyeSeries& pass : passes)
		{
			reportBlinks(tag, pass.passName, kEyeNames[sideIndex], *pass.series, faceSamples, sideIndex, chosenColumn,
						 chosen.lagMs);
		}
	}
}

EyeSeries eyeSeriesFromRatios(const std::vector<ModelSample>& samples)
{
	EyeSeries series;
	for (const ModelSample& sample : samples)
	{
		EyeSample eye;
		eye.timestampMs= sample.timestampMs;
		eye.bValid= sample.ratios.valid;
		eye.bModelFrame= true;
		eye.openness[0]= sample.ratios.eyeOpenness[0];
		eye.openness[1]= sample.ratios.eyeOpenness[1];
		series.push_back(eye);
	}
	return series;
}

// -- The blink evaluation's face models --------------------------------------

// A candidate gives up to two eye signals from one run (the MediaPipe mesh's
// eye points and its blendshape model's blink columns)
constexpr int kMaxEyeSignals= 2;
using EyeSignals= std::array<std::array<float, 2>, kMaxEyeSignals>;

// The face on the frame as the whole-body face block places it: the 68
// points' extent, which runs from the brows to the chin, and the roll of the
// line between the outer eye corners
struct FaceBox
{
	glm::vec2 center{0.f};
	float side= 0.f;
	float rollRadians= 0.f;
};

bool faceBoxFromBlock(const WholeBodyExtras& extras, FaceBox& outBox)
{
	if (!computeWholeBodyFaceRatios(extras).valid)
		return false;
	glm::vec2 boxMin(FLT_MAX);
	glm::vec2 boxMax(-FLT_MAX);
	for (const glm::vec3& point : extras.face)
	{
		boxMin= glm::min(boxMin, glm::vec2(point));
		boxMax= glm::max(boxMax, glm::vec2(point));
	}
	outBox.center= 0.5f * (boxMin + boxMax);
	outBox.side= std::max(boxMax.x - boxMin.x, boxMax.y - boxMin.y);
	// iBUG 36 and 45, the outer corners of the person's right and left eyes
	const glm::vec2 eyeLine= glm::vec2(extras.face[45]) - glm::vec2(extras.face[36]);
	outBox.rollRadians= std::atan2(eyeLine.y, eyeLine.x);
	return outBox.side > 8.f;
}

// A closed ring of eyelid points as an openness: its area over the square of
// its longest span, which needs no knowledge of where the ring starts
float ringOpenness(const glm::vec2* ring, int count)
{
	float area= 0.f;
	float span= 0.f;
	for (int index= 0; index < count; ++index)
	{
		const glm::vec2& a= ring[index];
		const glm::vec2& b= ring[(index + 1) % count];
		area+= a.x * b.y - b.x * a.y;
		for (int other= index + 1; other < count; ++other)
			span= std::max(span, glm::distance(a, ring[other]));
	}
	return span > 1e-3f ? 0.5f * std::fabs(area) / (span * span) : 0.f;
}

// An RTMPose face model, 106 points in the LaPa layout
struct RtmFaceCandidate
{
	RtmPoseModel model;

	bool load(const std::string& path) { return model.load(path, "directml"); }
	const char* activeEp() const { return model.activeEp(); }
	int signalCount() const { return 1; }
	const char* signalName(int) const { return "eyelid ring area over its span squared"; }

	bool estimate(const cv::Mat& image, const FaceBox& box, EyeSignals& outSignals)
	{
		// The model was trained on a face detector's box, which holds the
		// forehead the 68 points stop short of
		const float halfSide= 0.5f * 1.2f * box.side;
		const glm::vec2 center= box.center - glm::vec2(0.f, 0.1f * box.side);
		RtmPoseResult result;
		model.estimate(image, center - glm::vec2(halfSide), center + glm::vec2(halfSide), result);
		if (!result.valid || result.keypointCount != 106)
			return false;

		// LaPa's two eyelid rings, 66 to 73 and 75 to 82. The ring at the
		// larger image x is the person's left eye.
		static const int kRingStarts[2]= {66, 75};
		float openness[2]= {0.f, 0.f};
		float meanX[2]= {0.f, 0.f};
		for (int ringIndex= 0; ringIndex < 2; ++ringIndex)
		{
			glm::vec2 ring[8];
			for (int index= 0; index < 8; ++index)
			{
				ring[index]= glm::vec2(result.points[kRingStarts[ringIndex] + index]);
				meanX[ringIndex]+= ring[index].x / 8.f;
			}
			openness[ringIndex]= ringOpenness(ring, 8);
		}
		const int leftRing= meanX[0] > meanX[1] ? 0 : 1;
		outSignals[0][0]= openness[leftRing];
		outSignals[0][1]= openness[1 - leftRing];
		return true;
	}
};

// MediaPipe Face Landmarker's mesh model (478 points with the irises) and
// its blendshape model, which reads 146 of the mesh points
struct MediaPipeFaceCandidate
{
	static constexpr int kInputSize= 256;
	static constexpr int kLandmarkCount= 478;
	static constexpr int kSubsetCount= 146;
	// The blendshape model's outputs, in its own order
	static constexpr int kEyeBlinkLeft= 9;
	static constexpr int kEyeBlinkRight= 10;

	OnnxSession mesh;
	OnnxSession blendshapes;
	int landmarkOutput= -1;
	std::vector<float> inputBuffer;
	std::array<float, kSubsetCount * 2> subsetBuffer{};
	cv::Mat cropMat;
	cv::Mat rgbMat;
	// The mesh's outer eye corner span over the whole-body block's extent
	Stats widthRatio;

	bool load(const std::string& meshPath, const std::string& blendshapePath)
	{
		if (!mesh.create(meshPath, "directml") || !blendshapes.create(blendshapePath, "directml"))
			return false;
		landmarkOutput= mesh.findOutputByLastDim(kLandmarkCount * 3);
		inputBuffer.resize((size_t)kInputSize * kInputSize * 3);
		return landmarkOutput >= 0;
	}
	const char* activeEp() const { return mesh.activeEp(); }
	int signalCount() const { return 2; }
	const char* signalName(int signalIndex) const
	{
		return signalIndex == 0 ? "mesh eyelid gap over the eye's width" : "blendshape model, one minus eyeBlink";
	}

	bool estimate(const cv::Mat& image, const FaceBox& box, EyeSignals& outSignals)
	{
		// The landmark model's region: the face's box grown by half, square,
		// turned upright by the eye line
		const float scale= (float)kInputSize / (1.5f * box.side);
		const float c= std::cos(box.rollRadians);
		const float s= std::sin(box.rollRadians);
		const float half= 0.5f * (float)kInputSize;
		const cv::Matx23f imageToCrop(scale * c, scale * s, half - scale * (c * box.center.x + s * box.center.y),
									  -scale * s, scale * c, half - scale * (-s * box.center.x + c * box.center.y));
		cv::warpAffine(image, cropMat, imageToCrop, cv::Size(kInputSize, kInputSize), cv::INTER_LINEAR,
					   cv::BORDER_CONSTANT);
		cv::cvtColor(cropMat, rgbMat, cv::COLOR_BGR2RGB);
		cv::Mat inputMat(kInputSize, kInputSize, CV_32FC3, inputBuffer.data());
		rgbMat.convertTo(inputMat, CV_32FC3, 1.0 / 255.0);

		const std::array<int64_t, 4> inputShape= {1, kInputSize, kInputSize, 3};
		Ort::Value inputTensor= Ort::Value::CreateTensor<float>(OnnxSession::getCpuMemoryInfo(), inputBuffer.data(),
																inputBuffer.size(), inputShape.data(),
																inputShape.size());
		std::vector<Ort::Value> outputs= mesh.run(&inputTensor, 1);
		const float* landmarks= outputs[landmarkOutput].GetTensorData<float>();

		// Crop px back to frame px
		auto point= [&](int index) {
			const float x= (landmarks[index * 3] - half) / scale;
			const float y= (landmarks[index * 3 + 1] - half) / scale;
			return box.center + glm::vec2(c * x - s * y, s * x + c * y);
		};

		// Side 0, the person's left eye: corners 263 and 362. Side 1: 33 and 133.
		static const int kCorners[2][2]= {{263, 362}, {33, 133}};
		static const int kUpper[2][3]= {{387, 386, 385}, {160, 159, 158}};
		static const int kLower[2][3]= {{373, 374, 380}, {144, 145, 153}};
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const float width= glm::distance(point(kCorners[sideIndex][0]), point(kCorners[sideIndex][1]));
			float gap= 0.f;
			for (int index= 0; index < 3; ++index)
				gap+= glm::distance(point(kUpper[sideIndex][index]), point(kLower[sideIndex][index])) / 3.f;
			outSignals[0][sideIndex]= width > 1e-3f ? gap / width : 0.f;
		}
		widthRatio.add(glm::distance(point(263), point(33)) / std::max(box.side, 1.f));

		// The blendshape model reads the subset's x and y in frame px
		static const int kSubset[kSubsetCount]= {
			0,   1,   4,   5,   6,   7,   8,   10,  13,  14,  17,  21,  33,  37,  39,  40,  46,  52,  53,  54,  55,
			58,  61,  63,  65,  66,  67,  70,  78,  80,  81,  82,  84,  87,  88,  91,  93,  95,  103, 105, 107, 109,
			127, 132, 133, 136, 144, 145, 146, 148, 149, 150, 152, 153, 154, 155, 157, 158, 159, 160, 161, 162, 163,
			168, 172, 173, 176, 178, 181, 185, 191, 195, 197, 234, 246, 249, 251, 263, 267, 269, 270, 276, 282, 283,
			284, 285, 288, 291, 293, 295, 296, 297, 300, 308, 310, 311, 312, 314, 317, 318, 321, 323, 324, 332, 334,
			336, 338, 356, 361, 362, 365, 373, 374, 375, 377, 378, 379, 380, 381, 382, 384, 385, 386, 387, 388, 389,
			390, 397, 398, 400, 402, 405, 409, 415, 454, 466, 468, 469, 470, 471, 472, 473, 474, 475, 476, 477};
		for (int index= 0; index < kSubsetCount; ++index)
		{
			const glm::vec2 px= point(kSubset[index]);
			subsetBuffer[(size_t)index * 2]= px.x;
			subsetBuffer[(size_t)index * 2 + 1]= px.y;
		}
		const std::array<int64_t, 3> subsetShape= {1, kSubsetCount, 2};
		Ort::Value subsetTensor= Ort::Value::CreateTensor<float>(OnnxSession::getCpuMemoryInfo(), subsetBuffer.data(),
																 subsetBuffer.size(), subsetShape.data(),
																 subsetShape.size());
		std::vector<Ort::Value> scores= blendshapes.run(&subsetTensor, 1);
		const float* blend= scores[0].GetTensorData<float>();
		outSignals[1][0]= 1.f - blend[kEyeBlinkLeft];
		outSignals[1][1]= 1.f - blend[kEyeBlinkRight];
		return true;
	}
};

struct FaceCandidatePass
{
	std::array<EyeSeries, kMaxEyeSignals> series;
	Stats estimateMs;
	int framesRead= 0;
	int framesWithFace= 0;
};

// The candidate over the window's stored frames, each frame's crop from the
// whole-body face block of the observation the list holds on it
template <typename Candidate>
void runFaceCandidatePass(Candidate& candidate, const std::string& frameDirectory, int cameraIndex,
						  const std::vector<FrameObservation>& frames, double startMs, double endMs,
						  FaceCandidatePass& outPass)
{
	for (const FrameObservation& frame : frames)
	{
		if (frame.timestampMs < startMs || frame.timestampMs > endMs)
			continue;
		const cv::Mat image=
			cv::imread(FrameRecorder::makeFramePath(frameDirectory, cameraIndex, frame.frameIndex), cv::IMREAD_COLOR);
		if (image.empty())
			continue;
		outPass.framesRead++;

		EyeSignals signals{};
		FaceBox box;
		bool bRead= false;
		if (frame.body.valid && faceBoxFromBlock(frame.body.wholeBody, box))
		{
			const auto start= std::chrono::steady_clock::now();
			bRead= candidate.estimate(image, box, signals);
			outPass.estimateMs.add(
				(float)std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count());
		}
		outPass.framesWithFace+= bRead ? 1 : 0;
		for (int signalIndex= 0; signalIndex < candidate.signalCount(); ++signalIndex)
		{
			EyeSample sample;
			sample.timestampMs= frame.timestampMs;
			sample.bValid= bRead;
			sample.bModelFrame= frame.bModelFrame;
			sample.openness[0]= signals[signalIndex][0];
			sample.openness[1]= signals[signalIndex][1];
			outPass.series[signalIndex].push_back(sample);
		}
	}
}

// A candidate at the three cadences it could run at beside the body model:
// every camera frame with the body model also on every frame, every camera
// frame with the crop held from the body model's last frame at the divider,
// and the divider's frames alone
template <typename Candidate>
void reportFaceCandidate(const char* tag, const char* name, Candidate& candidate, const std::string& frameDirectory,
						 int cameraIndex, const std::vector<FrameObservation>& everyFrames,
						 const std::vector<FrameObservation>& dividedFrames, double startMs, double endMs,
						 const std::vector<RecordedFaceSample>& faceSamples, const std::string& seriesCsvPath)
{
	FaceCandidatePass fresh;
	FaceCandidatePass held;
	runFaceCandidatePass(candidate, frameDirectory, cameraIndex, everyFrames, startMs, endMs, fresh);
	runFaceCandidatePass(candidate, frameDirectory, cameraIndex, dividedFrames, startMs, endMs, held);
	if (!seriesCsvPath.empty())
	{
		// The every-frame pass, one row per frame
		std::ofstream csv(seriesCsvPath, std::ios::trunc);
		csv << "timestampMs,valid";
		for (int signalIndex= 0; signalIndex < candidate.signalCount(); ++signalIndex)
			csv << ",signal" << signalIndex << "Left,signal" << signalIndex << "Right";
		csv << "\n";
		for (size_t row= 0; row < fresh.series[0].size(); ++row)
		{
			csv << std::fixed << std::setprecision(3) << fresh.series[0][row].timestampMs << ","
				<< (fresh.series[0][row].bValid ? 1 : 0) << std::setprecision(5);
			for (int signalIndex= 0; signalIndex < candidate.signalCount(); ++signalIndex)
			{
				csv << "," << fresh.series[signalIndex][row].openness[0] << ","
					<< fresh.series[signalIndex][row].openness[1];
			}
			csv << "\n";
		}
	}
	MIKAN_LOG_INFO(tag) << name << " (" << candidate.activeEp() << "): read on "
						<< percent(held.framesWithFace, held.framesRead) << "% of " << held.framesRead
						<< " frames, crop and inference p50 " << held.estimateMs.p50() << " ms, p95 "
						<< held.estimateMs.percentile(0.95f) << " ms";
	for (int signalIndex= 0; signalIndex < candidate.signalCount(); ++signalIndex)
	{
		EyeSeries atDivider;
		for (const EyeSample& sample : held.series[signalIndex])
		{
			if (sample.bModelFrame)
				atDivider.push_back(sample);
		}
		MIKAN_LOG_INFO(tag) << " " << name << ", " << candidate.signalName(signalIndex) << ":";
		reportBlinkSignal(tag,
						  {{"every frame", &fresh.series[signalIndex]},
						   {"every frame, crop held from the divider", &held.series[signalIndex]},
						   {"at the divider", &atDivider}},
						  faceSamples);
	}
}

// -- Hands against the stereo rig ------------------------------------------

struct CameraGeometry
{
	glm::dmat4 cameraFromWorld{1.0};
	float fx= 0.f, fy= 0.f, cx= 0.f, cy= 0.f;

	bool project(const glm::vec3& world, glm::vec2& outPx) const
	{
		const glm::dvec4 camera= cameraFromWorld * glm::dvec4(glm::dvec3(world), 1.0);
		if (camera.z < 1e-3)
			return false;
		outPx= glm::vec2((float)(fx * camera.x / camera.z) + cx, (float)(fy * camera.y / camera.z) + cy);
		return true;
	}
};

// The points a fused hand and an image hand share: the wrist and the index,
// middle and pinky knuckles
constexpr int kSharedPointCount= 4;
constexpr int kSharedLandmarks[kSharedPointCount]= {0, 5, 9, 17};
constexpr eFinger kSharedFingers[kSharedPointCount]= {eFinger::Middle, eFinger::Index, eFinger::Middle,
													   eFinger::Pinky};

struct TruthHand
{
	bool valid= false;
	std::array<glm::vec2, kSharedPointCount> points{};
	glm::vec2 palm{0.f};
	// The wrist to the middle knuckle, px
	float palmLengthPx= 0.f;
};

TruthHand makeTruthHand(const HandPose& pose, const CameraGeometry& geometry, int frameWidth, int frameHeight)
{
	TruthHand truth;
	// Triangulated hands only: a fused hand that one camera alone supplied is
	// that camera's own slot, a scene false positive included, and the camera
	// under test may be the one
	if (!pose.tracked || !pose.hasWorldPose || !pose.stereoTriangulated)
		return truth;
	bool bProjected= geometry.project(pose.getWristPositionWorld(), truth.points[0]);
	for (int index= 1; index < kSharedPointCount; ++index)
	{
		const glm::vec3 world=
			pose.palmPositionWorld + pose.palmOrientationWorld * pose.skeleton.baseInPalm[(int)kSharedFingers[index]];
		bProjected= bProjected && geometry.project(world, truth.points[index]);
	}
	bProjected= bProjected && geometry.project(pose.palmPositionWorld, truth.palm);
	truth.palmLengthPx= glm::length(truth.points[2] - truth.points[0]);
	truth.valid= bProjected && truth.palmLengthPx > 1.f && truth.palm.x >= 0.f && truth.palm.y >= 0.f &&
		truth.palm.x < (float)frameWidth && truth.palm.y < (float)frameHeight;
	return truth;
}

struct Box
{
	glm::vec2 min{FLT_MAX};
	glm::vec2 max{-FLT_MAX};
	void add(const glm::vec2& p)
	{
		min= glm::min(min, p);
		max= glm::max(max, p);
	}
	bool contains(const glm::vec2& p) const { return p.x >= min.x && p.y >= min.y && p.x <= max.x && p.y <= max.y; }
	float area() const { return std::max(max.x - min.x, 0.f) * std::max(max.y - min.y, 0.f); }
};

template <typename Points> Box boxOf(const Points& points)
{
	Box box;
	for (const glm::vec3& point : points)
		box.add(glm::vec2(point));
	return box;
}

float intersectionOverUnion(const Box& a, const Box& b)
{
	Box overlap;
	overlap.min= glm::max(a.min, b.min);
	overlap.max= glm::min(a.max, b.max);
	const float intersection= overlap.area();
	const float unionArea= a.area() + b.area() - intersection;
	return unionArea > 0.f ? intersection / unionArea : 0.f;
}

struct SideVerdicts
{
	int truthCount= 0;
	int decided= 0;
	int agreed= 0;
	void add(int side, int truthSide)
	{
		truthCount++;
		if (side < 0)
			return;
		decided++;
		agreed+= side == truthSide ? 1 : 0;
	}
	void log(const char* tag, const char* name) const
	{
		MIKAN_LOG_INFO(tag) << "    " << name << ": decided " << decided << " of " << truthCount << " ("
							<< percent(decided, truthCount) << "%), right on " << percent(agreed, decided)
							<< "% of those, " << percent(agreed, truthCount) << "% of all";
	}
};

void reportHands(const char* tag, const TrackingReplay& replay, const std::vector<ModelSample>& samples,
				 const CameraGeometry& geometry)
{
	std::array<Stats, 2> pointErrorPx[kSharedPointCount];
	std::array<Stats, 2> pointErrorPalms[kSharedPointCount];
	int truthFrames[2]= {0, 0};
	int scoredFrames[2]= {0, 0};
	int boxHoldsPalm[2]= {0, 0};
	Stats slotOverlap;
	int slotOverlapHalf= 0;
	SideVerdicts shapeAll, wristAll, shapeClose, wristClose;
	int slotsWithoutTruth= 0;
	// Each slot's distance to the fused hand nearest it, palm lengths: the
	// rig's own agreement with the camera, which bounds every number below
	Stats slotTruthDistance;

	for (const ModelSample& sample : samples)
	{
		const TrackingReplay::ReplayFrame& frame= replay.getFrame(sample.replayFrameIndex);
		const WholeBodyExtras& extras= sample.body.wholeBody;

		TruthHand truth[2];
		bool bHandScored[2]= {false, false};
		Box wholeBodyBox[2];
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			truth[sideIndex]= makeTruthHand(frame.replayedFused.poses[sideIndex], geometry, sample.frameWidth, sample.frameHeight);
			wholeBodyBox[sideIndex]= boxOf(extras.hands[sideIndex]);
			bHandScored[sideIndex]= true;
			for (int landmark : kSharedLandmarks)
				bHandScored[sideIndex]= bHandScored[sideIndex] && extras.handScores[sideIndex][landmark] >= kScoreGate;
			if (!truth[sideIndex].valid)
				continue;

			truthFrames[sideIndex]++;
			if (!bHandScored[sideIndex])
				continue;
			scoredFrames[sideIndex]++;
			boxHoldsPalm[sideIndex]+= wholeBodyBox[sideIndex].contains(truth[sideIndex].palm) ? 1 : 0;
			for (int index= 0; index < kSharedPointCount; ++index)
			{
				const float error= glm::length(
					glm::vec2(extras.hands[sideIndex][kSharedLandmarks[index]]) - truth[sideIndex].points[index]);
				pointErrorPx[index][sideIndex].add(error);
				pointErrorPalms[index][sideIndex].add(error / truth[sideIndex].palmLengthPx);
			}
		}

		const glm::vec2 shoulderLeft(sample.body.imagePoints[(int)ePoseLandmark::LEFT_SHOULDER]);
		const glm::vec2 shoulderRight(sample.body.imagePoints[(int)ePoseLandmark::RIGHT_SHOULDER]);
		const float shoulderWidthPx= glm::length(shoulderLeft - shoulderRight);
		const bool bHandsClose= truth[0].valid && truth[1].valid &&
			glm::length(truth[0].points[0] - truth[1].points[0]) <= kHandsCloseShoulderWidths * shoulderWidthPx;

		for (const RecordedHandInput& slot : sample.slots)
		{
			if (!slot.tracked)
				continue;

			// The slot's fused side: the fused hand whose shared points it
			// lies on, clearly nearer than the other
			float truthDistance[2]= {FLT_MAX, FLT_MAX};
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				if (!truth[sideIndex].valid)
					continue;
				float sum= 0.f;
				for (int index= 0; index < kSharedPointCount; ++index)
				{
					sum+= glm::length(glm::vec2(slot.imagePoints[kSharedLandmarks[index]]) -
									  truth[sideIndex].points[index]);
				}
				truthDistance[sideIndex]= sum / (float)kSharedPointCount / truth[sideIndex].palmLengthPx;
			}
			const int truthSide= truthDistance[0] <= truthDistance[1] ? 0 : 1;
			if (truthDistance[truthSide] < FLT_MAX)
				slotTruthDistance.add(truthDistance[truthSide]);
			if (truthDistance[truthSide] > 1.f ||
				truthDistance[truthSide] > kShapeMarginRatio * truthDistance[1 - truthSide])
			{
				slotsWithoutTruth++;
				continue;
			}

			const Box slotBox= boxOf(slot.imagePoints);
			if (bHandScored[truthSide])
			{
				const float overlap= intersectionOverUnion(slotBox, wholeBodyBox[truthSide]);
				slotOverlap.add(overlap);
				slotOverlapHalf+= overlap >= 0.5f ? 1 : 0;
			}

			// By shape, the rule the direct solver's label report reads
			const int shapeSide= matchHandSideByShape(slot.imagePoints, extras).side;

			// By the wrist: the slot's wrist within the agreement distance of
			// one body wrist and not the other
			int wristSide= -1;
			{
				const glm::vec2 slotWrist(slot.imagePoints[0]);
				const int wristLandmarks[2]= {(int)ePoseLandmark::LEFT_WRIST, (int)ePoseLandmark::RIGHT_WRIST};
				bool bNear[2]= {false, false};
				for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
				{
					bNear[sideIndex]= sample.body.visibility[wristLandmarks[sideIndex]] >= 0.5f &&
						glm::length(slotWrist - glm::vec2(sample.body.imagePoints[wristLandmarks[sideIndex]])) <=
							kHandAgreementShoulderWidths * shoulderWidthPx;
				}
				if (bNear[0] != bNear[1])
					wristSide= bNear[0] ? 0 : 1;
			}

			shapeAll.add(shapeSide, truthSide);
			wristAll.add(wristSide, truthSide);
			if (bHandsClose)
			{
				shapeClose.add(shapeSide, truthSide);
				wristClose.add(wristSide, truthSide);
			}
		}
	}

	static const char* kSideNames[2]= {"left", "right"};
	static const char* kPointNames[kSharedPointCount]= {"wrist", "index knuckle", "middle knuckle", "pinky knuckle"};
	MIKAN_LOG_INFO(tag) << "whole-body hands against the fused hands projected into the camera:";
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		MIKAN_LOG_INFO(tag) << "  " << kSideNames[sideIndex] << ": fused hand in view on " << truthFrames[sideIndex]
							<< " model frames, the whole-body hand scored (>= " << kScoreGate << ") on "
							<< percent(scoredFrames[sideIndex], truthFrames[sideIndex]) << "%, its box holds the palm on "
							<< percent(boxHoldsPalm[sideIndex], scoredFrames[sideIndex]) << "% of those ("
							<< percent(boxHoldsPalm[sideIndex], truthFrames[sideIndex]) << "% of all)";
		for (int index= 0; index < kSharedPointCount; ++index)
		{
			MIKAN_LOG_INFO(tag) << "    " << kPointNames[index] << ": p50 " << pointErrorPx[index][sideIndex].p50()
								<< " px (" << pointErrorPalms[index][sideIndex].p50() << " palm lengths), p90 "
								<< pointErrorPx[index][sideIndex].p90() << " px ("
								<< pointErrorPalms[index][sideIndex].p90() << ")";
		}
	}
	MIKAN_LOG_INFO(tag) << "  hand slot to the fused hand nearest it: p50 " << slotTruthDistance.p50()
						<< " palm lengths, p90 " << slotTruthDistance.p90() << " (" << slotTruthDistance.count()
						<< " slot frames)";
	MIKAN_LOG_INFO(tag) << "  hand slot box against the whole-body hand's box: overlap p50 " << slotOverlap.p50()
						<< ", at least half on " << percent(slotOverlapHalf, slotOverlap.count()) << "% of "
						<< slotOverlap.count() << " slot frames";
	MIKAN_LOG_INFO(tag) << "  the hand slot's side against the fused side (" << slotsWithoutTruth
						<< " slot frames on no fused hand left out):";
	shapeAll.log(tag, "by shape, every frame");
	wristAll.log(tag, "by wrist, every frame");
	shapeClose.log(tag, "by shape, fused hands close together");
	wristClose.log(tag, "by wrist, fused hands close together");
}

// -- The head against the phone head ---------------------------------------

// A generic face's six points in a camera-like frame (x to the image's right,
// y down, z away from the viewer) for a face looking straight at the camera,
// so a fit of a frontal face is near the identity. Arbitrary units: only the
// rotation is read.
struct FaceModelPoint
{
	int landmark; // iBUG index
	cv::Point3f position;
};
const FaceModelPoint kSixPointFace[]= {
	{30, {0.f, 0.f, 0.f}},         // nose tip
	{8, {0.f, 330.f, 65.f}},       // chin
	{36, {-225.f, -170.f, 135.f}}, // right eye outer corner
	{45, {225.f, -170.f, 135.f}},  // left eye outer corner
	{48, {-150.f, 150.f, 125.f}},  // right mouth corner
	{54, {150.f, 150.f, 125.f}},   // left mouth corner
};

bool fitHeadSixPoint(const WholeBodyExtras& extras, const CameraGeometry& geometry, glm::quat& outRotation)
{
	if (!extras.present)
		return false;
	std::vector<cv::Point3f> objectPoints;
	std::vector<cv::Point2f> imagePoints;
	for (const FaceModelPoint& point : kSixPointFace)
	{
		if (extras.faceScores[point.landmark] < kScoreGate)
			return false;
		objectPoints.push_back(point.position);
		imagePoints.emplace_back(extras.face[point.landmark].x, extras.face[point.landmark].y);
	}
	const cv::Mat cameraMatrix=
		(cv::Mat_<double>(3, 3) << geometry.fx, 0.0, geometry.cx, 0.0, geometry.fy, geometry.cy, 0.0, 0.0, 1.0);
	cv::Mat rvec, tvec;
	if (!cv::solvePnP(objectPoints, imagePoints, cameraMatrix, cv::noArray(), rvec, tvec, false, cv::SOLVEPNP_SQPNP))
		return false;
	if (tvec.at<double>(2) <= 0.0)
		return false;
	cv::Mat rotation;
	cv::Rodrigues(rvec, rotation);
	glm::mat3 matrix;
	for (int row= 0; row < 3; ++row)
	{
		for (int column= 0; column < 3; ++column)
			matrix[column][row]= (float)rotation.at<double>(row, column);
	}
	outRotation= glm::normalize(glm::quat_cast(matrix));
	return true;
}

// The rotation that best carries one point set onto another, both about
// their own centroids
glm::quat rigidRotation(const std::vector<glm::vec3>& from, const std::vector<glm::vec3>& to)
{
	glm::vec3 fromCentroid(0.f);
	glm::vec3 toCentroid(0.f);
	for (size_t index= 0; index < from.size(); ++index)
	{
		fromCentroid+= from[index];
		toCentroid+= to[index];
	}
	fromCentroid/= (float)from.size();
	toCentroid/= (float)from.size();

	cv::Mat covariance= cv::Mat::zeros(3, 3, CV_64F);
	for (size_t index= 0; index < from.size(); ++index)
	{
		const glm::vec3 a= from[index] - fromCentroid;
		const glm::vec3 b= to[index] - toCentroid;
		for (int row= 0; row < 3; ++row)
		{
			for (int column= 0; column < 3; ++column)
				covariance.at<double>(row, column)+= (double)b[row] * (double)a[column];
		}
	}
	cv::Mat singular, u, vt;
	cv::SVD::compute(covariance, singular, u, vt);
	cv::Mat flip= cv::Mat::eye(3, 3, CV_64F);
	flip.at<double>(2, 2)= cv::determinant(u * vt) < 0.0 ? -1.0 : 1.0;
	const cv::Mat rotation= u * flip * vt;
	glm::mat3 matrix;
	for (int row= 0; row < 3; ++row)
	{
		for (int column= 0; column < 3; ++column)
			matrix[column][row]= (float)rotation.at<double>(row, column);
	}
	return glm::normalize(glm::quat_cast(matrix));
}

// The face block's rigid points: the brows, the nose and the eyes. The jaw
// line slides along the silhouette as the head turns and the lips move with
// speech.
constexpr int kRigidFaceFirst= 17;
constexpr int kRigidFaceLast= 47;
constexpr int kRigidFaceCount= kRigidFaceLast - kRigidFaceFirst + 1;

// The rigid face points in camera space, meters, from the model's own depth:
// each pixel's ray at the depth the shoulders' pixel width implies for the
// body, plus the point's depth past the shoulders', times a gain on the
// model's depth. False when a point or a shoulder scores under the gate.
bool liftRigidFace(const BodyPoseObservation& body, const CameraGeometry& geometry, float shoulderWidthMeters,
				   float depthGain, std::vector<glm::vec3>& outPoints)
{
	outPoints.clear();
	if (!body.wholeBody.present || !body.hasDepth)
		return false;
	const int leftShoulder= (int)ePoseLandmark::LEFT_SHOULDER;
	const int rightShoulder= (int)ePoseLandmark::RIGHT_SHOULDER;
	if (body.visibility[leftShoulder] < kScoreGate || body.visibility[rightShoulder] < kScoreGate)
		return false;
	const float shoulderWidthPx=
		glm::length(glm::vec2(body.imagePoints[leftShoulder]) - glm::vec2(body.imagePoints[rightShoulder]));
	if (shoulderWidthPx < 1.f)
		return false;
	const float shoulderDepth= geometry.fx * shoulderWidthMeters / shoulderWidthPx;
	const float shoulderModelDepth= 0.5f * (body.imagePoints[leftShoulder].z + body.imagePoints[rightShoulder].z);

	for (int index= kRigidFaceFirst; index <= kRigidFaceLast; ++index)
	{
		if (body.wholeBody.faceScores[index] < kScoreGate)
			return false;
		const glm::vec3& point= body.wholeBody.face[index];
		const float depth= shoulderDepth + depthGain * (point.z - shoulderModelDepth);
		outPoints.emplace_back((point.x - geometry.cx) / geometry.fx * depth,
							   (point.y - geometry.cy) / geometry.fy * depth, depth);
	}
	return true;
}

glm::vec3 rotationVector(glm::quat q)
{
	if (q.w < 0.f)
		q= -q;
	const glm::vec3 axis(q.x, q.y, q.z);
	const float sine= glm::length(axis);
	if (sine < 1e-6f)
		return glm::vec3(0.f);
	return axis / sine * (2.f * atan2f(sine, q.w));
}

struct HeadSample
{
	double timestampMs= 0.0;
	bool bPhone= false;
	glm::quat phone{1.f, 0.f, 0.f, 0.f};
	// The methods: the six-point face fit, the direct solver's ear and nose
	// head while it reads one, and the rigid fit of the face's own lifted
	// points at each depth gain
	static constexpr int kMethodCount= 5;
	bool bMethod[kMethodCount]= {};
	glm::quat method[kMethodCount];
};

void reportHeadMethod(const char* tag, const char* name, const std::vector<HeadSample>& samples, int methodIndex,
					  double restStartMs, double restEndMs, double earsStartMs, double earsEndMs,
					  double headphonesStartMs, double headphonesEndMs)
{
	// Both heads as turns from the rest pose. The phone's turn and the
	// camera's are the same turn seen in two frames, so one fixed rotation
	// carries the phone's rotation vectors onto the method's.
	const HeadSample* reference= nullptr;
	for (const HeadSample& sample : samples)
	{
		if (reference == nullptr && sample.bPhone && sample.bMethod[methodIndex] &&
			sample.timestampMs >= restStartMs && sample.timestampMs <= restEndMs)
			reference= &sample;
	}
	if (reference == nullptr)
	{
		MIKAN_LOG_INFO(tag) << "  " << name << ": no frame in the rest window with this head and the phone's";
		return;
	}

	struct Turn
	{
		double timestampMs;
		glm::vec3 phone;
		glm::vec3 method;
	};
	std::vector<Turn> turns;
	int phoneFrames[2]= {0, 0};
	int methodFrames[2]= {0, 0};
	// 0 the ears-visible window, 1 the headphones window, -1 neither
	auto windowOf= [&](double timestampMs) {
		if (timestampMs >= headphonesStartMs && timestampMs <= headphonesEndMs)
			return 1;
		return timestampMs >= earsStartMs && timestampMs <= earsEndMs ? 0 : -1;
	};
	for (const HeadSample& sample : samples)
	{
		const int window= windowOf(sample.timestampMs);
		if (!sample.bPhone || window < 0)
			continue;
		phoneFrames[window]++;
		if (!sample.bMethod[methodIndex])
			continue;
		methodFrames[window]++;
		// Only the windows' frames shape the alignment: a hand over the face
		// elsewhere in the take is not a head turn
		turns.push_back({sample.timestampMs, rotationVector(sample.phone * glm::inverse(reference->phone)),
						 rotationVector(sample.method[methodIndex] * glm::inverse(reference->method[methodIndex]))});
	}

	cv::Mat covariance= cv::Mat::zeros(3, 3, CV_64F);
	for (const Turn& turn : turns)
	{
		for (int row= 0; row < 3; ++row)
		{
			for (int column= 0; column < 3; ++column)
				covariance.at<double>(row, column)+= (double)turn.method[row] * (double)turn.phone[column];
		}
	}
	cv::Mat singular, u, vt;
	cv::SVD::compute(covariance, singular, u, vt);
	// The best orthogonal map, and the best rotation. They differ when the
	// best map is a mirror, which is what a phone axis read with the wrong
	// sign looks like: its turns are then the mirror image of the camera's,
	// and no rotation carries one set onto the other.
	const bool bMirror= cv::determinant(u * vt) < 0.0;
	cv::Mat flip= cv::Mat::eye(3, 3, CV_64F);
	flip.at<double>(2, 2)= bMirror ? -1.0 : 1.0;
	auto toGlm= [](const cv::Mat& matrix) {
		glm::mat3 out;
		for (int row= 0; row < 3; ++row)
		{
			for (int column= 0; column < 3; ++column)
				out[column][row]= (float)matrix.at<double>(row, column);
		}
		return out;
	};
	const glm::mat3 methodFromPhone= toGlm(u * flip * vt);
	const glm::mat3 methodFromPhoneMirrored= toGlm(u * vt);

	static const char* kWindowNames[2]= {"ears visible", "headphones"};
	Stats error[2];
	Stats errorMirrored[2];
	Stats errorTurned[2];
	Stats turnRatio[2];
	for (const Turn& turn : turns)
	{
		const int window= windowOf(turn.timestampMs);
		auto toQuat= [](const glm::vec3& vector) {
			const float angle= glm::length(vector);
			return angle > 1e-6f ? glm::angleAxis(angle, vector / angle) : glm::quat(1.f, 0.f, 0.f, 0.f);
		};
		auto errorAgainst= [&](const glm::mat3& map) {
			return glm::degrees(
				glm::length(rotationVector(toQuat(turn.method) * glm::inverse(toQuat(map * turn.phone)))));
		};
		const float errorDegrees= errorAgainst(methodFromPhone);
		error[window].add(errorDegrees);
		errorMirrored[window].add(errorAgainst(methodFromPhoneMirrored));
		const float phoneDegrees= glm::degrees(glm::length(turn.phone));
		if (phoneDegrees >= 10.f)
		{
			errorTurned[window].add(errorDegrees);
			turnRatio[window].add(glm::degrees(glm::length(turn.method)) / phoneDegrees);
		}
	}
	for (int window= 0; window < 2; ++window)
	{
		if (phoneFrames[window] == 0)
			continue;
		MIKAN_LOG_INFO(tag) << "  " << name << ", " << kWindowNames[window] << ": read on "
							<< percent(methodFrames[window], phoneFrames[window]) << "% of " << phoneFrames[window]
							<< " frames, error p50 " << error[window].p50() << " deg p90 " << error[window].p90()
							<< ", with the head turned 10 deg or more (" << errorTurned[window].count() << " frames) p50 "
							<< errorTurned[window].p50() << " p90 " << errorTurned[window].p90()
							<< ", its turn over the phone's p50 " << turnRatio[window].p50()
							<< (bMirror ? "; the best map is a MIRROR, error under it p50 " : "; no mirror fits better, p50 ")
							<< errorMirrored[window].p50();
	}
}

// -- The hand pipeline with body hints --------------------------------------

// One run of the hand pipeline over the camera's stored frames, as the
// direct camera runs it: no cross-camera hints, and with body hints the
// previous frame's observation seeding the slots under the live retry
// throttle
struct HintRunStats
{
	// Per body side: frames with the triangulated hand in view, those with a
	// tracked slot on it, and the runs of frames without one
	int truthFrames[2]= {0, 0};
	int coveredFrames[2]= {0, 0};
	Stats dropoutFrames[2];
	// Tracked slots on neither triangulated hand, on frames with both in view
	int strayFrames= 0;
	int bothTruthFrames= 0;
	// Against the body model alone, which needs no rig: tracked slots whose
	// box touches neither whole-body hand's, and per side the frames with the
	// whole-body hand scored and those with a slot's box touching it
	int slotFrames= 0;
	int offBodySlotFrames= 0;
	int bodyHandFrames[2]= {0, 0};
	int bodyHandSlotFrames[2]= {0, 0};
	Stats pipelineMs;
	HandSeedStats seedStats;
};

bool runHintPass(const char* tag, const TrackingReplay& replay, const std::string& frameDirectory, int cameraIndex,
				 const std::vector<FrameObservation>& frames, const CameraGeometry* truthGeometry, bool bBodyHints,
				 HintRunStats& outStats, std::ofstream* seedCsv= nullptr)
{
	const AppConfig& config= replay.getRecordedConfig();
	HandTrackingPipelineConfig pipelineConfig;
	pipelineConfig.flipHandedness= config.tracking.flipHandedness;
	pipelineConfig.detectorIntervalFrames= config.tracking.detectorIntervalFrames;
	pipelineConfig.palmScoreThresholdRelaxed= config.tracking.palmScoreThresholdRelaxed;
	pipelineConfig.preferredEp= config.tracking.onnxEp;
	HandTrackingPipeline pipeline;
	if (!pipeline.startup(pipelineConfig))
	{
		MIKAN_LOG_ERROR(tag) << "Failed to start the hand pipeline";
		return false;
	}

	BodyPoseObservation lastBody;
	int lastWidth= 0;
	int lastHeight= 0;
	int hintCooldownFrames= 0;
	int dropoutRun[2]= {0, 0};
	std::vector<HandSearchHint> hints;
	std::vector<HandBox> handBoxes;
	bool bHintsSet= false;
	HandBodySeedStats seedBefore;
	TrackingFrameResult prevResult;
	if (seedCsv != nullptr)
	{
		*seedCsv << "timestampMs,side,centerX,centerY,palmPx,edgePx,gatedScore,meanScore,wristVisibility,"
					"onTrackedSlot,slotsBefore,slotsAfter,applied,evicted,accepted,rejected,redundant,noFreeSlot,"
					"tooSmall\n";
	}
	for (const FrameObservation& frame : frames)
	{
		const cv::Mat image=
			cv::imread(FrameRecorder::makeFramePath(frameDirectory, cameraIndex, frame.frameIndex), cv::IMREAD_COLOR);
		if (image.empty())
			continue;

		if (bBodyHints)
		{
			if (hintCooldownFrames > 0)
			{
				hintCooldownFrames--;
			}
			else
			{
				makeBodyHandHints(lastBody, lastWidth, lastHeight, hints, handBoxes);
				if (!hints.empty())
				{
					pipeline.setSearchHints(hints, handBoxes);
					hintCooldownFrames= 2;
					bHintsSet= true;
				}
			}
		}

		TrackingFrameResult result;
		pipeline.process(image, result);
		outStats.pipelineMs.add(result.inferenceMs);

		// One row per hint handed over on this frame, with what the frame's
		// funnel did. A frame's two hints share the funnel's counts.
		const HandBodySeedStats seedNow= pipeline.getSeedStats().body;
		if (seedCsv != nullptr && bHintsSet)
		{
			std::vector<HandBox> slotBoxesBefore;
			for (const TrackedHand& hand : prevResult.hands)
			{
				if (!hand.tracked)
					continue;
				const Box box= boxOf(hand.imagePoints);
				slotBoxesBefore.push_back({box.min, box.max});
			}
			int slotsAfter= 0;
			for (const TrackedHand& hand : result.hands)
				slotsAfter+= hand.tracked ? 1 : 0;
			for (const HandSearchHint& hint : hints)
			{
				// The whole-body hand the hint came from: the nearer palm center
				int hintSide= 0;
				float nearest= FLT_MAX;
				for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
				{
					const auto& hand= lastBody.wholeBody.hands[sideIndex];
					const glm::vec2 palmCenter= 0.5f * (glm::vec2(hand[0]) + glm::vec2(hand[9]));
					const float distance= glm::distance(palmCenter, hint.centerPx);
					if (distance < nearest)
					{
						nearest= distance;
						hintSide= sideIndex;
					}
				}
				float gatedScore= 1.f;
				for (int landmark : kSharedLandmarks)
					gatedScore= std::min(gatedScore, lastBody.wholeBody.handScores[hintSide][landmark]);
				float meanScore= 0.f;
				for (float score : lastBody.wholeBody.handScores[hintSide])
					meanScore+= score / (float)WHOLEBODY_HAND_LANDMARK_COUNT;
				const float edgePx= std::min(std::min(hint.centerPx.x, (float)lastWidth - hint.centerPx.x),
											 std::min(hint.centerPx.y, (float)lastHeight - hint.centerPx.y));
				const int wristLandmark= hintSide == 0 ? (int)ePoseLandmark::LEFT_WRIST : (int)ePoseLandmark::RIGHT_WRIST;
				*seedCsv << std::fixed << std::setprecision(3) << frame.timestampMs << std::defaultfloat
						 << std::setprecision(5) << "," << hintSide << "," << hint.centerPx.x << "," << hint.centerPx.y
						 << "," << hint.palmSizePx << "," << edgePx << "," << gatedScore << "," << meanScore << ","
						 << lastBody.visibility[wristLandmark] << ","
						 << (HandTrackingPipeline::isSeedRedundant(hint.centerPx, slotBoxesBefore) ? 1 : 0) << ","
						 << slotBoxesBefore.size() << "," << slotsAfter << ","
						 << seedNow.applied - seedBefore.applied << "," << seedNow.evicted - seedBefore.evicted << ","
						 << seedNow.accepted - seedBefore.accepted << ","
						 << seedNow.rejectedByModel - seedBefore.rejectedByModel << ","
						 << seedNow.skippedRedundant - seedBefore.skippedRedundant << ","
						 << seedNow.skippedNoFreeSlot - seedBefore.skippedNoFreeSlot << ","
						 << seedNow.skippedTooSmall - seedBefore.skippedTooSmall << "\n";
			}
		}
		seedBefore= seedNow;
		bHintsSet= false;
		prevResult= result;
		lastBody= frame.body;
		lastWidth= image.cols;
		lastHeight= image.rows;

		// Against this frame's whole-body hands. One slot's box off both of
		// them is what chooseSlotToEvict reports for a pair of that slot.
		std::vector<HandSearchHint> frameHints;
		std::vector<HandBox> frameHandBoxes;
		makeBodyHandHints(frame.body, image.cols, image.rows, frameHints, frameHandBoxes);
		if (frameHandBoxes.size() == 2)
		{
			HandBox slotBoxes[2];
			for (int slotIndex= 0; slotIndex < 2; ++slotIndex)
			{
				const TrackedHand& hand= result.hands[slotIndex];
				if (!hand.tracked)
					continue;
				const Box box= boxOf(hand.imagePoints);
				slotBoxes[slotIndex]= {box.min, box.max};
				outStats.slotFrames++;
				outStats.offBodySlotFrames+=
					HandTrackingPipeline::chooseSlotToEvict({slotBoxes[slotIndex], slotBoxes[slotIndex]}, {1.f, 1.f},
															frameHandBoxes) >= 0
					? 1
					: 0;
			}
			for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
			{
				bool bScored= true;
				for (int landmark : kSharedLandmarks)
				{
					bScored= bScored &&
						frame.body.wholeBody.handScores[sideIndex][landmark] >= k_bodyHandHintScoreGate;
				}
				if (!bScored)
					continue;
				outStats.bodyHandFrames[sideIndex]++;
				bool bSlotOnIt= false;
				for (int slotIndex= 0; slotIndex < 2; ++slotIndex)
				{
					bSlotOnIt= bSlotOnIt ||
						(result.hands[slotIndex].tracked &&
						 HandTrackingPipeline::chooseSlotToEvict({slotBoxes[slotIndex], slotBoxes[slotIndex]},
																 {1.f, 1.f}, {frameHandBoxes[sideIndex]}) < 0);
				}
				outStats.bodyHandSlotFrames[sideIndex]+= bSlotOnIt ? 1 : 0;
			}
		}

		if (truthGeometry == nullptr)
			continue;
		const TrackingReplay::ReplayFrame& replayFrame= replay.getFrame(frame.replayFrameIndex);
		TruthHand truth[2];
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			truth[sideIndex]=
				makeTruthHand(replayFrame.replayedFused.poses[sideIndex], *truthGeometry, image.cols, image.rows);
		}

		// A slot is on a triangulated hand within a palm length over the
		// shared points
		auto slotOn= [&](const TrackedHand& hand, int sideIndex) {
			if (!hand.tracked || !truth[sideIndex].valid)
				return false;
			float sum= 0.f;
			for (int index= 0; index < kSharedPointCount; ++index)
			{
				sum+= glm::length(glm::vec2(hand.imagePoints[kSharedLandmarks[index]]) -
								  truth[sideIndex].points[index]);
			}
			return sum / (float)kSharedPointCount <= truth[sideIndex].palmLengthPx;
		};

		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			if (!truth[sideIndex].valid)
			{
				// The hand left the rig's view: the run ends unmeasured
				dropoutRun[sideIndex]= 0;
				continue;
			}
			outStats.truthFrames[sideIndex]++;
			if (slotOn(result.hands[0], sideIndex) || slotOn(result.hands[1], sideIndex))
			{
				outStats.coveredFrames[sideIndex]++;
				if (dropoutRun[sideIndex] > 0)
					outStats.dropoutFrames[sideIndex].add((float)dropoutRun[sideIndex]);
				dropoutRun[sideIndex]= 0;
			}
			else
			{
				dropoutRun[sideIndex]++;
			}
		}
		if (truth[0].valid && truth[1].valid)
		{
			outStats.bothTruthFrames++;
			for (const TrackedHand& hand : result.hands)
				outStats.strayFrames+= hand.tracked && !slotOn(hand, 0) && !slotOn(hand, 1) ? 1 : 0;
		}
	}
	outStats.seedStats= pipeline.getSeedStats();
	return true;
}

void logHintRun(const char* tag, const char* name, const HintRunStats& stats)
{
	static const char* kSideNames[2]= {"left", "right"};
	MIKAN_LOG_INFO(tag) << "  " << name << ": pipeline p50 " << stats.pipelineMs.p50() << " ms p90 "
						<< stats.pipelineMs.p90() << ", slots off both whole-body hands " << stats.offBodySlotFrames
						<< " of " << stats.slotFrames << " slot frames ("
						<< percent(stats.offBodySlotFrames, stats.slotFrames) << "%)";
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		MIKAN_LOG_INFO(tag) << "    " << kSideNames[sideIndex] << " whole-body hand: a slot on it for "
							<< stats.bodyHandSlotFrames[sideIndex] << " of " << stats.bodyHandFrames[sideIndex]
							<< " scored frames (" << percent(stats.bodyHandSlotFrames[sideIndex], stats.bodyHandFrames[sideIndex])
							<< "%)";
	}
	if (stats.truthFrames[0] + stats.truthFrames[1] > 0)
	{
		MIKAN_LOG_INFO(tag) << "    against the triangulated hands: slots on neither " << stats.strayFrames
							<< " over the " << stats.bothTruthFrames << " frames with both in view";
	}
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (stats.truthFrames[sideIndex] == 0)
			continue;
		const Stats& dropouts= stats.dropoutFrames[sideIndex];
		MIKAN_LOG_INFO(tag) << "    " << kSideNames[sideIndex] << ": a slot on the hand for "
							<< stats.coveredFrames[sideIndex] << " of " << stats.truthFrames[sideIndex] << " frames ("
							<< percent(stats.coveredFrames[sideIndex], stats.truthFrames[sideIndex]) << "%), "
							<< dropouts.count() << " dropouts that ended, frames to the slot's return p50 "
							<< dropouts.p50() << " p90 " << dropouts.p90() << " longest " << dropouts.percentile(1.f);
	}
	const HandBodySeedStats& body= stats.seedStats.body;
	if (body.offered > 0)
	{
		MIKAN_LOG_INFO(tag) << "    body hints: offered " << body.offered << ", on a tracked hand "
							<< body.skippedRedundant << ", on a place just turned down " << body.skippedBackoff
							<< ", too small " << body.skippedTooSmall << ", no slot to take "
							<< body.skippedNoFreeSlot << ", seeded " << body.applied << " (" << body.evicted
							<< " by eviction), the model kept " << body.accepted << " and dropped "
							<< body.rejectedByModel;
	}
}

// -- Per-frame CSV ---------------------------------------------------------

void writeCsv(const std::string& path, const std::vector<ModelSample>& samples,
			  const std::vector<RecordedFaceSample>& faceSamples)
{
	static const char* kColumns[]= {"eyeBlinkLeft",    "eyeBlinkRight", "jawOpen",     "mouthSmileLeft",
									"mouthSmileRight", "mouthPucker",   "browInnerUp", "browOuterUpLeft",
									"browOuterUpRight"};
	std::ofstream csv(path, std::ios::trunc);
	csv << "timestampMs,valid,minScore,faceWidthPx,eyeOpennessLeft,eyeOpennessRight,mouthOpen,mouthWidth,"
		   "mouthCornerLiftLeft,mouthCornerLiftRight,browHeightLeft,browHeightRight,browInnerHeight";
	for (const char* column : kColumns)
		csv << ",phone_" << column;
	csv << "\n";
	for (const ModelSample& sample : samples)
	{
		const WholeBodyFaceRatios& ratios= sample.ratios;
		// The clock's milliseconds run to nine digits before the point
		csv << std::fixed << std::setprecision(3) << sample.timestampMs << std::defaultfloat << std::setprecision(6)
			<< "," << (ratios.valid ? 1 : 0) << "," << ratios.minScore << ","
			<< ratios.faceWidthPx << "," << ratios.eyeOpenness[0] << "," << ratios.eyeOpenness[1] << ","
			<< ratios.mouthOpen << "," << ratios.mouthWidth << "," << ratios.mouthCornerLift[0] << ","
			<< ratios.mouthCornerLift[1] << "," << ratios.browHeight[0] << "," << ratios.browHeight[1] << ","
			<< ratios.browInnerHeight;
		for (const char* column : kColumns)
		{
			// The phone at the frame's own time, unlagged; -1 where it has no face
			float value= 0.f;
			const bool bHave= phoneValueAt(faceSamples, sample.timestampMs, arkitBlendshapeFromName(column), value);
			csv << "," << (bHave ? value : -1.f);
		}
		csv << "\n";
	}
}
} // namespace

static int runGateWholeBodyTool(const TestArgs& args)
{
	const char* tag= "gate-wholebody";

	std::string recordingPath;
	std::string csvPath;
	int requestedCamera= -1;
	// Windows of the take, seconds from its first frame
	struct Window
	{
		const char* key;
		double startSeconds;
		double endSeconds;
	};
	Window rest= {"rest=", 0.0, 3.0};
	Window shapes= {"shapes=", 0.0, 1e9};
	Window ears= {"ears=", 0.0, 1e9};
	Window headphones= {"headphones=", -1.0, -1.0};
	// The phone's clock ahead of the camera's for the head comparison, ms
	double headLagMs= 0.0;
	const char* usage= "Usage: --gate-wholebody <recording.jsonl> [cameraIndex] [rest=<s>-<s>] [shapes=<s>-<s>] "
					   "[ears=<s>-<s>] [headphones=<s>-<s>] [headlag=<ms>] [frames.csv]";
	for (const std::string& arg : args)
	{
		Window* window= nullptr;
		for (Window* candidate : {&rest, &shapes, &ears, &headphones})
		{
			if (arg.rfind(candidate->key, 0) == 0)
				window= candidate;
		}
		if (TrackingRecording::isRecordingFile(arg))
			recordingPath= arg;
		else if (arg.size() > 4 && arg.compare(arg.size() - 4, 4, ".csv") == 0)
			csvPath= arg;
		else if (arg.rfind("headlag=", 0) == 0)
			headLagMs= atof(arg.substr(8).c_str());
		else if (window != nullptr)
		{
			const size_t keyLength= strlen(window->key);
			const size_t dash= arg.find('-', keyLength);
			if (dash == std::string::npos)
			{
				MIKAN_LOG_ERROR(tag) << window->key << " takes a range of seconds, as " << window->key << "0-3";
				return 1;
			}
			window->startSeconds= atof(arg.substr(keyLength, dash - keyLength).c_str());
			window->endSeconds= atof(arg.substr(dash + 1).c_str());
		}
		else
			requestedCamera= atoi(arg.c_str());
	}
	if (recordingPath.empty())
	{
		MIKAN_LOG_ERROR(tag) << usage;
		return 1;
	}

	TrackingReplay replay;
	std::string error;
	if (!replay.load(recordingPath, error))
	{
		MIKAN_LOG_ERROR(tag) << recordingPath << ": load failed: " << error;
		return 1;
	}
	const AppConfig& config= replay.getRecordedConfig();

	int cameraIndex= requestedCamera;
	if (cameraIndex < 0 && config.direct.enabled)
		cameraIndex= config.direct.cameraIndex;
	for (int index= 0; cameraIndex < 0 && index < (int)config.cameraCount(); ++index)
	{
		if (config.camera(index).bodyPose.enabled)
			cameraIndex= index;
	}
	if (cameraIndex < 0 || cameraIndex >= (int)config.cameraCount())
	{
		MIKAN_LOG_ERROR(tag) << "No direct or body-pose camera to default to; pass a cameraIndex";
		return 1;
	}
	const CameraProfile& profile= config.camera(cameraIndex);

	const std::string frameDirectory= TrackingRecording::makeFrameDirectoryPath(recordingPath);
	if (!std::filesystem::exists(frameDirectory))
	{
		MIKAN_LOG_ERROR(tag) << "No raw frames beside the recording (" << frameDirectory << ")";
		return 1;
	}

	// The every-frame pass, and the pass at the cadence the live stage runs
	const int liveDivider= std::max(profile.bodyPose.poseFrameDivider, 2);
	std::vector<ModelSample> everyFrame;
	std::vector<ModelSample> divided;
	// Every stored frame with the observation the live stage would hold on it
	std::vector<FrameObservation> dividedFrames;
	Stats inferenceMs;
	Stats dividedInferenceMs;
	int framesMissing= 0;
	int dividedFramesMissing= 0;
	std::vector<FrameObservation> everyFrames;
	if (!runModelPass(tag, replay, frameDirectory, cameraIndex, 1, everyFrame, inferenceMs, framesMissing,
					  &everyFrames) ||
		!runModelPass(tag, replay, frameDirectory, cameraIndex, liveDivider, divided, dividedInferenceMs,
					  dividedFramesMissing, &dividedFrames))
		return 1;
	if (everyFrame.empty())
	{
		MIKAN_LOG_ERROR(tag) << "No frame of camera " << cameraIndex << " gave a model result (missing frames "
							 << framesMissing << ")";
		return 1;
	}

	const double takeStartMs= everyFrame.front().timestampMs;
	const double takeSeconds= (everyFrame.back().timestampMs - takeStartMs) / 1000.0;
	int faceValid= 0;
	Stats faceWidth;
	Stats faceMinScore;
	for (const ModelSample& sample : everyFrame)
	{
		if (!sample.ratios.valid)
			continue;
		faceValid++;
		faceWidth.add(sample.ratios.faceWidthPx);
		faceMinScore.add(sample.ratios.minScore);
	}
	MIKAN_LOG_INFO(tag) << std::filesystem::path(recordingPath).filename().string() << ", camera " << cameraIndex
						<< ": " << everyFrame.size() << " model frames over " << takeSeconds << " s (missing frames "
						<< framesMissing << "), " << divided.size() << " at divider " << liveDivider
						<< ", inference p50 " << inferenceMs.p50() << " ms";
	MIKAN_LOG_INFO(tag) << "face block: read on " << percent(faceValid, (int)everyFrame.size())
						<< "% of model frames, face width p50 " << faceWidth.p50() << " px (p10 "
						<< faceWidth.percentile(0.1f) << "), lowest score among the points read p50 "
						<< faceMinScore.p50() << " (p10 " << faceMinScore.percentile(0.1f) << ")";

	const double restStartMs= takeStartMs + rest.startSeconds * 1000.0;
	const double restEndMs= takeStartMs + rest.endSeconds * 1000.0;
	reportFaceJitter(tag, everyFrame, restStartMs, restEndMs);

	// The shapes are read over their own window of the take, so a head sweep
	// or a hand over the face elsewhere in it does not count against them
	auto inShapesWindow= [&](const std::vector<ModelSample>& pass) {
		std::vector<ModelSample> window;
		for (const ModelSample& sample : pass)
		{
			const double seconds= (sample.timestampMs - takeStartMs) / 1000.0;
			if (seconds >= shapes.startSeconds && seconds <= shapes.endSeconds)
				window.push_back(sample);
		}
		return window;
	};
	const std::vector<ModelSample> shapesEveryFrame= inShapesWindow(everyFrame);
	const std::vector<ModelSample> shapesDivided= inShapesWindow(divided);

	// -- Face shapes against the phone -------------------------------------
	const std::vector<RecordedFaceSample>& faceSamples= replay.getFaceSamples();
	int phoneTracked= 0;
	int phoneWithHead= 0;
	for (const RecordedFaceSample& sample : faceSamples)
	{
		phoneTracked+= sample.bTracked ? 1 : 0;
		phoneWithHead+= sample.bHasHead ? 1 : 0;
	}
	if (phoneTracked == 0)
	{
		MIKAN_LOG_INFO(tag) << "face shapes: no face sidecar with a tracked face beside this take, not measured";
	}
	else
	{
		MIKAN_LOG_INFO(tag) << "phone: " << faceSamples.size() << " samples, face tracked on "
							<< percent(phoneTracked, (int)faceSamples.size()) << "%, head rotation on "
							<< percent(phoneWithHead, (int)faceSamples.size()) << "%";

		auto column= [](const char* name) { return arkitBlendshapeFromName(name); };
		const ShapeGetter eyeLeft= [](const WholeBodyFaceRatios& r) { return r.eyeOpenness[0]; };
		const ShapeGetter eyeRight= [](const WholeBodyFaceRatios& r) { return r.eyeOpenness[1]; };
		const ShapeGetter mouthOpen= [](const WholeBodyFaceRatios& r) { return r.mouthOpen; };
		const ShapeGetter mouthWidth= [](const WholeBodyFaceRatios& r) { return r.mouthWidth; };
		const ShapeGetter liftLeft= [](const WholeBodyFaceRatios& r) { return r.mouthCornerLift[0]; };
		const ShapeGetter liftRight= [](const WholeBodyFaceRatios& r) { return r.mouthCornerLift[1]; };
		const ShapeGetter browLeft= [](const WholeBodyFaceRatios& r) { return r.browHeight[0]; };
		const ShapeGetter browRight= [](const WholeBodyFaceRatios& r) { return r.browHeight[1]; };
		const ShapeGetter browInner= [](const WholeBodyFaceRatios& r) { return r.browInnerHeight; };

		struct Pairing
		{
			const char* shapeName;
			ShapeGetter getter;
			const char* columnName;
		};
		const Pairing pairings[]= {
			{"left eye openness", eyeLeft, "eyeBlinkLeft"},
			{"left eye openness", eyeLeft, "eyeBlinkRight"},
			{"right eye openness", eyeRight, "eyeBlinkRight"},
			{"right eye openness", eyeRight, "eyeBlinkLeft"},
			{"mouth open", mouthOpen, "jawOpen"},
			{"mouth width", mouthWidth, "mouthSmileLeft"},
			{"mouth width", mouthWidth, "mouthSmileRight"},
			{"mouth width", mouthWidth, "mouthPucker"},
			{"left corner lift", liftLeft, "mouthSmileLeft"},
			{"left corner lift", liftLeft, "mouthSmileRight"},
			{"right corner lift", liftRight, "mouthSmileRight"},
			{"right corner lift", liftRight, "mouthSmileLeft"},
			{"inner brow height", browInner, "browInnerUp"},
			{"left brow height", browLeft, "browOuterUpLeft"},
			{"left brow height", browLeft, "browOuterUpRight"},
			{"right brow height", browRight, "browOuterUpRight"},
			{"right brow height", browRight, "browOuterUpLeft"},
		};
		for (int passIndex= 0; passIndex < 2; ++passIndex)
		{
			const std::vector<ModelSample>& pass= passIndex == 0 ? shapesEveryFrame : shapesDivided;
			MIKAN_LOG_INFO(tag) << "face shapes against the phone, " << (passIndex == 0 ? "every frame" : "at the divider")
								<< ":";
			for (const Pairing& pairing : pairings)
			{
				logCorrelation(tag, pairing.shapeName, pairing.columnName,
							   correlate(pass, faceSamples, pairing.getter, column(pairing.columnName)));
			}
		}

		// Each eye against the phone column it follows best, at that lag
		MIKAN_LOG_INFO(tag) << "blinks (a phone blink is its column past " << kPhoneBlinkOnLevel << ", a fast one under "
							<< kFastBlinkMs << " ms):";
		MIKAN_LOG_INFO(tag) << " whole-body points, eyelid gap over the eye's width:";
		const EyeSeries pointsEveryFrame= eyeSeriesFromRatios(shapesEveryFrame);
		const EyeSeries pointsDivided= eyeSeriesFromRatios(shapesDivided);
		reportBlinkSignal(tag, {{"every frame", &pointsEveryFrame}, {"at the divider", &pointsDivided}}, faceSamples);

		// The face model candidates, each on a crop the face block places.
		// A candidate whose model file is not beside the app is not run.
		const double windowStartMs= takeStartMs + shapes.startSeconds * 1000.0;
		const double windowEndMs= takeStartMs + std::min(shapes.endSeconds, 1e7) * 1000.0;
		static const char* kRtmFaceNames[2]= {"RTMPose face6-m", "RTMPose face6-t"};
		static const char* kRtmFacePaths[2]= {"models/rtmpose_face6_m.onnx", "models/rtmpose_face6_t.onnx"};
		static const char* kRtmFaceSuffixes[2]= {"_rtmpose_m", "_rtmpose_t"};
		// With a frames CSV, each candidate's every-frame series beside it
		auto seriesCsvPath= [&](const char* suffix) {
			if (csvPath.empty())
				return std::string();
			const std::filesystem::path path(csvPath);
			return (path.parent_path() / (path.stem().string() + suffix + ".csv")).string();
		};
		for (int modelIndex= 0; modelIndex < 2; ++modelIndex)
		{
			RtmFaceCandidate candidate;
			if (!std::filesystem::exists(kRtmFacePaths[modelIndex]) || !candidate.load(kRtmFacePaths[modelIndex]))
			{
				MIKAN_LOG_INFO(tag) << kRtmFaceNames[modelIndex] << ": " << kRtmFacePaths[modelIndex]
									<< " not found or not loadable, not run";
				continue;
			}
			reportFaceCandidate(tag, kRtmFaceNames[modelIndex], candidate, frameDirectory, cameraIndex, everyFrames,
								dividedFrames, windowStartMs, windowEndMs, faceSamples,
								seriesCsvPath(kRtmFaceSuffixes[modelIndex]));
		}
		{
			const char* meshPath= "models/mediapipe_face_landmarks.onnx";
			const char* blendshapePath= "models/mediapipe_face_blendshapes.onnx";
			MediaPipeFaceCandidate candidate;
			if (!std::filesystem::exists(meshPath) || !std::filesystem::exists(blendshapePath) ||
				!candidate.load(meshPath, blendshapePath))
			{
				MIKAN_LOG_INFO(tag) << "MediaPipe Face Landmarker: " << meshPath << " and " << blendshapePath
									<< " not found or not loadable, not run";
			}
			else
			{
				reportFaceCandidate(tag, "MediaPipe Face Landmarker", candidate, frameDirectory, cameraIndex,
									everyFrames, dividedFrames, windowStartMs, windowEndMs, faceSamples,
									seriesCsvPath("_mediapipe"));
				MIKAN_LOG_INFO(tag) << " MediaPipe mesh eye corner span over the face block's extent p50 "
									<< candidate.widthRatio.p50() << " (blendshape model on "
									<< candidate.blendshapes.activeEp() << ")";
			}
		}
	}

	CameraGeometry geometry;
	if (profile.intrinsics.present)
	{
		const MikanMatrix3d& cameraMatrix= profile.intrinsics.intrinsics.undistorted_camera_matrix;
		geometry.fx= (float)cameraMatrix.x0;
		geometry.fy= (float)cameraMatrix.y1;
		geometry.cx= (float)cameraMatrix.z0;
		geometry.cy= (float)cameraMatrix.z1;
	}

	// -- The head against the phone head -----------------------------------
	if (phoneWithHead == 0 || !profile.intrinsics.present)
	{
		MIKAN_LOG_INFO(tag) << "head: " << (phoneWithHead == 0 ? "the face sidecar carries no phone head" : "no intrinsics")
							<< ", not measured";
	}
	else
	{
		// The direct solver's own head on the same observations, under the
		// recording's direct config on the shared synthetic rig
		const AvatarModel avatarModel= SyntheticAvatar::makeModel(eVrmVersion::Vrm1, false);
		const AvatarSkeleton skeleton(avatarModel);
		const DirectRetargetConfig directConfig= makeDirectRetargetConfig(config, AvatarRigSettings());
		DirectStageSolver direct;
		DirectCameraModel cameraModel;
		cameraModel.fx= geometry.fx;
		cameraModel.fy= geometry.fy;
		cameraModel.cx= geometry.cx;
		cameraModel.cy= geometry.cy;

		// The rigid fit's template per depth gain: the lifted face's mean
		// shape over the rest window, about its centroid. 1 is the model's
		// depth as it comes, the others shrink it.
		constexpr int kDepthGainCount= 3;
		const float depthGains[kDepthGainCount]= {0.25f, 0.5f, 1.f};
		const float shoulderWidthMeters= makeBodyDimensions(config).shoulderWidthMeters;
		std::vector<glm::vec3> templates[kDepthGainCount];
		for (int gainIndex= 0; gainIndex < kDepthGainCount; ++gainIndex)
		{
			std::vector<glm::vec3> sum(kRigidFaceCount, glm::vec3(0.f));
			std::vector<glm::vec3> lifted;
			int count= 0;
			for (const ModelSample& sample : everyFrame)
			{
				if (sample.timestampMs < restStartMs || sample.timestampMs > restEndMs ||
					!liftRigidFace(sample.body, geometry, shoulderWidthMeters, depthGains[gainIndex], lifted))
					continue;
				glm::vec3 centroid(0.f);
				for (const glm::vec3& point : lifted)
					centroid+= point;
				centroid/= (float)kRigidFaceCount;
				for (int index= 0; index < kRigidFaceCount; ++index)
					sum[index]+= lifted[index] - centroid;
				count++;
			}
			if (count > 0)
			{
				for (glm::vec3& point : sum)
					point/= (float)count;
				templates[gainIndex]= sum;
			}
		}

		std::vector<HeadSample> headSamples;
		std::vector<glm::vec3> liftedFace;
		for (const ModelSample& sample : everyFrame)
		{
			HeadSample head;
			head.timestampMs= sample.timestampMs;
			const RecordedFaceSample* phone=
				FaceSidecar::findAtOrBefore(faceSamples, sample.timestampMs + headLagMs);
			if (phone != nullptr && phone->bTracked && phone->bHasHead &&
				sample.timestampMs + headLagMs - phone->timestampMs <= kPhoneStaleMs)
			{
				head.bPhone= true;
				head.phone= faceHeadDeltaFromPhoneEuler(phone->headEulerDegrees);
			}

			head.bMethod[0]= fitHeadSixPoint(sample.body.wholeBody, geometry, head.method[0]);

			TrackingFrameResult cameraInput;
			cameraInput.timestampMs= sample.timestampMs;
			cameraInput.frameWidth= sample.frameWidth;
			cameraInput.frameHeight= sample.frameHeight;
			cameraInput.body= sample.body;
			cameraModel.width= sample.frameWidth;
			cameraModel.height= sample.frameHeight;
			AvatarPose pose;
			direct.solve(cameraInput, cameraModel, TrackingFrameResult::HeadPose(), sample.timestampMs, skeleton,
						 directConfig, pose);
			const DirectRetarget::SolveState& state= direct.getSolveState();
			// Measured or within its hold, as the live head reads
			head.bMethod[1]= state.headMeasured;
			head.method[1]= state.headFrame;

			for (int gainIndex= 0; gainIndex < kDepthGainCount; ++gainIndex)
			{
				if (templates[gainIndex].empty() ||
					!liftRigidFace(sample.body, geometry, shoulderWidthMeters, depthGains[gainIndex], liftedFace))
					continue;
				head.bMethod[2 + gainIndex]= true;
				head.method[2 + gainIndex]= rigidRotation(templates[gainIndex], liftedFace);
			}
			headSamples.push_back(head);
		}

		const double earsStartMs= takeStartMs + ears.startSeconds * 1000.0;
		const double earsEndMs= takeStartMs + ears.endSeconds * 1000.0;
		const double headphonesStartMs= takeStartMs + headphones.startSeconds * 1000.0;
		const double headphonesEndMs= takeStartMs + headphones.endSeconds * 1000.0;
		MIKAN_LOG_INFO(tag) << "head against the phone head (phone lag " << headLagMs
							<< " ms), as turns from the rest pose after the one rotation that best carries the phone's "
							   "turns onto the method's:";
		reportHeadMethod(tag, "six-point face fit", headSamples, 0, restStartMs, restEndMs, earsStartMs, earsEndMs,
						 headphonesStartMs, headphonesEndMs);
		reportHeadMethod(tag, "ear and nose head", headSamples, 1, restStartMs, restEndMs, earsStartMs, earsEndMs,
						 headphonesStartMs, headphonesEndMs);
		// The two camera heads against each other, no phone in it: the ear
		// and nose head stands where the phone's did
		{
			std::vector<HeadSample> pair= headSamples;
			for (HeadSample& sample : pair)
			{
				sample.bPhone= sample.bMethod[1];
				sample.phone= sample.method[1];
			}
			reportHeadMethod(tag, "six-point face fit against the ear and nose head", pair, 0, restStartMs, restEndMs,
							 earsStartMs, earsEndMs, headphonesStartMs, headphonesEndMs);
		}
		static const char* kGainNames[kDepthGainCount]= {"model-depth rigid fit, depth times 0.25",
														   "model-depth rigid fit, depth times 0.5",
														   "model-depth rigid fit, depth as it comes"};
		for (int gainIndex= 0; gainIndex < kDepthGainCount; ++gainIndex)
		{
			reportHeadMethod(tag, kGainNames[gainIndex], headSamples, 2 + gainIndex, restStartMs, restEndMs, earsStartMs,
							 earsEndMs, headphonesStartMs, headphonesEndMs);
		}
	}

	// -- Hands against the stereo rig --------------------------------------
	if (!profile.intrinsics.present || !profile.extrinsics.present)
	{
		MIKAN_LOG_INFO(tag) << "hands: camera " << cameraIndex
							<< " has no intrinsics or extrinsics, so no fused hand to project into it, not measured";
	}
	else
	{
		geometry.cameraFromWorld= glm::inverse(profile.extrinsics.markerFromCamera);
		replay.runAll();
		reportHands(tag, replay, divided, geometry);
	}

	// -- The hand pipeline as the direct camera runs it, without and with
	// the body model's hints ------------------------------------------------
	{
		const bool bHaveTruth= profile.intrinsics.present && profile.extrinsics.present;
		HintRunStats withoutHints;
		HintRunStats withHints;
		// With a frames CSV, the hints handed over beside it, one row each
		std::ofstream seedCsv;
		if (!csvPath.empty())
		{
			const std::filesystem::path path(csvPath);
			seedCsv.open((path.parent_path() / (path.stem().string() + "_seeds.csv")).string(), std::ios::trunc);
		}
		if (!runHintPass(tag, replay, frameDirectory, cameraIndex, dividedFrames, bHaveTruth ? &geometry : nullptr,
						 false, withoutHints) ||
			!runHintPass(tag, replay, frameDirectory, cameraIndex, dividedFrames, bHaveTruth ? &geometry : nullptr,
						 true, withHints, seedCsv.is_open() ? &seedCsv : nullptr))
			return 1;
		MIKAN_LOG_INFO(tag) << "hand pipeline re-run over the raw frames with no cross-camera hints:";
		logHintRun(tag, "without body hints", withoutHints);
		logHintRun(tag, "with body hints", withHints);
	}

	if (!csvPath.empty())
	{
		writeCsv(csvPath, everyFrame, faceSamples);
		MIKAN_LOG_INFO(tag) << "wrote " << everyFrame.size() << " rows to " << csvPath;
	}
	return 0;
}

MIKAN_REGISTER_TEST("--gate-wholebody",
					"Whole-body face and hand blocks on one camera against the phone face and the stereo rig's hands",
					eTestCategory::Tool, runGateWholeBodyTool);
