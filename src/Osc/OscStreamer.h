#pragma once

#include "OscWriter.h"
#include "TrackingTypes.h"
#include "UdpSocket.h"
#include "VmcRetarget.h"
#include "FaceProfileBinding.h"
#include "RetargetStage.h"

#include <atomic>
#include <chrono>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

struct TrackingFrameResult;

struct OscStreamerConfig
{
	bool enabled= true;
	std::string targetIp= "127.0.0.1";
	// The conventional VMC receiver port
	uint16_t port= 39539;
	float maxRateHz= 60.f; // <= 0 disables rate limiting
	// Wrist-to-elbow distance used to place the streamed elbow. Only the
	// length is assumed; the direction is measured, so an error here slides
	// the elbow along the forearm rather than rotating it.
	float forearmLengthMeters= 0.25f;

	// Bone offsets the streamed skeleton carries. A VMC receiver replaces both
	// the rotation AND the translation of every bone it is sent, so these are
	// not optional: the avatar takes these proportions.
	float shoulderWidthMeters= 0.40f;
	float upperArmLengthMeters= 0.30f;
	// Neck -> head bone offset. Nothing on this rig measures it, so it is a
	// setting: too small and the head sinks into the neck, too large and it
	// floats.
	float vmcHeadOffsetMeters= 0.08f;
	// With a loaded avatar the VMC bones come from the retarget stage's
	// posed avatar, measured against this skeleton (its proportions, its rest
	// hand), instead of the measured-length chain above. Null keeps the
	// measured-length path.
	std::shared_ptr<const AvatarSkeleton> avatarSkeleton;
	// The loaded avatar file, announced as /VMC/Ext/VRM so a receiver on the
	// same machine can load the very file the bones were retargeted onto.
	// An empty path announces nothing.
	std::string avatarPath;
	std::string avatarTitle;
	std::string avatarSha256;
	// The face profile binding whose targets name the blendshapes. A frame's
	// face values go out only when they come from this binding's generation.
	// Null sends no blendshapes.
	std::shared_ptr<const FaceProfileBinding> faceBinding;
};

/// Streams per-frame tracked poses as the VMC protocol (OSC 1.0 bundles over
/// UDP unicast) for VMC receivers. See VmcRetarget.h for the retarget and its
/// conventions. A frame is split into complete bundles of at most
/// k_maxDatagramBytes each; the messages, in order:
///   /VMC/Ext/OK ,iiii loaded calibrationState calibrationMode trackingStatus
///   /VMC/Ext/T ,f seconds since the socket opened
///   /VMC/Ext/Root/Pos ,sfffffff "root" + identity. Deliberately identity:
///     this is an upper-body tracker anchored to a desk marker, not a
///     room-scale root, so pushing the marker frame onto the avatar would
///     teleport it.
///   /VMC/Ext/Bone/Pos ,sfffffff name + local position xyz + rotation xyzw,
///     once per measured bone (head, clavicles, arms, hands, fingers, and
///     the spine, chest, upper chest and neck when a direct pose places them)
///   while either face source is live (the phone stream or the microphone),
///   the face profile's targets (see FaceProfileBinding for the names):
///     /VMC/Ext/Blend/Val ,sf name + value [0,1], once per target name, the
///       max of the outputs on it. A morph-target output goes out under its
///       morph name.
///     /VMC/Ext/Blend/Apply , after the last value. The frame both sources
///       stop sends every value once more at zero, so the face relaxes
///       instead of holding its last expression, and a frame after a
///       binding swap sends zero once for the names the new binding lacks.
///   /VMC/Ext/VRM ,sss path title sha256 (at most once per second, only with
///     an avatar loaded): the local file path (UTF-8), the avatar's title,
///     and the lowercase hex SHA-256 of the file bytes
class OscStreamer
{
public:
	OscStreamer()= default;
	~OscStreamer();

	// Non-copyable
	OscStreamer(const OscStreamer&)= delete;
	OscStreamer& operator=(const OscStreamer&)= delete;

	/// Open the UDP socket. @returns true on success
	bool startup();
	void shutdown();

	bool isRunning() const { return m_isRunning; }

	OscStreamerConfig getConfig() const;
	void setConfig(const OscStreamerConfig& config);

	/// Encode and send one bundle for the given frame, past the send-rate
	/// gate. Called from the inference thread; allocation-light after warm-up
	/// (reuses a pooled bundle and a scratch encode buffer).
	/// retarget is what the retarget stage made of the frame, its loss rules
	/// already applied (the confidence gate, the dropout hold, the freeze, the
	/// ease into rest): the streamer only encodes it. A valid pose streams
	/// through the configured avatar skeleton, and with no skeleton the frame
	/// carries no bones; the OK tracking status is 1 when either upper arm is
	/// present in the pose. An invalid pose streams the measured-length chain
	/// over the stage's resolved hands, the OK tracking status 1 when either
	/// is carried. The blendshapes and the OK, T, Root and VRM messages go out
	/// either way.
	void sendFrame(const TrackingFrameResult& frame, const RetargetResult& retarget);

	/// Encode one frame without touching the socket, advancing the same
	/// streaming state (the face and avatar throttles). sendFrame is this
	/// plus the rate gate and the send, so a test reads exactly the bytes a
	/// receiver would rather than a reconstruction of them.
	/// One entry per datagram: a frame too large for a single packet is split
	/// into several complete bundles (see k_maxDatagramBytes).
	/// retarget has the meaning it has on sendFrame.
	void encodeFrame(const TrackingFrameResult& frame, const RetargetResult& retarget,
					 std::vector<std::vector<uint8_t>>& outPackets);

	/// Largest datagram the streamer will put on the wire. Sized to stay inside
	/// a 1500-byte ethernet MTU (so nothing depends on IP fragmentation, where
	/// one lost fragment costs the whole bundle) and well inside the 2048-byte
	/// default receive buffer of Rug.Osc, which several VMC tools are built on.
	static constexpr size_t k_maxDatagramBytes= 1400;

	/// Frames sent per second (updated once a second). Safe to poll from the
	/// UI thread.
	float getMessagesPerSecond() const { return m_messagesPerSecond.load(std::memory_order_relaxed); }
	/// Datagrams and total bytes of the last frame sendFrame put on the wire.
	/// Safe to poll from any thread.
	int getLastPacketCount() const { return m_lastPacketCount.load(std::memory_order_relaxed); }
	int getLastFrameBytes() const { return m_lastFrameBytes.load(std::memory_order_relaxed); }

private:
	using ClockTimePoint= std::chrono::steady_clock::time_point;

	void encodeFrameLocked(const TrackingFrameResult& frame, const RetargetResult& retarget,
						   const ClockTimePoint& now, std::vector<std::vector<uint8_t>>& outPackets);
	void appendVmcMessages(const TrackingFrameResult& frame, const RetargetResult& retarget,
						   const ClockTimePoint& now);
	void appendVmcBlendMessages(const TrackingFrameResult& frame);
	void appendVmcAvatarMessage(const ClockTimePoint& now);
	void updateSendStats(const ClockTimePoint& now);

	mutable std::mutex m_mutex; // guards config, socket, and encode state
	OscStreamerConfig m_config;
	UdpSocket m_socket;
	bool m_isRunning= false;

	// Per-frame encode state (reused to stay allocation-light)
	OscBundle m_bundle;
	std::vector<std::vector<uint8_t>> m_scratchPackets;

	// The VMC bone scratch (reused so the per-frame encode stays
	// allocation-light)
	VmcRetarget::VmcPose m_vmcPose;
	ClockTimePoint m_startTime;
	// Face blendshape state: the wire scratch, whether the last frame carried
	// a face (for the one relaxing frame), and the names it sent (zeroed once
	// when a new binding drops them)
	std::vector<float> m_blendValues;
	bool m_bFaceWasPresent= false;
	std::vector<std::string> m_sentBlendNames;
	// /VMC/Ext/VRM throttling (wall clock)
	bool m_hasSentAvatar= false;
	ClockTimePoint m_lastAvatarTime;

	// Rate decimation (frame timestamps)
	double m_lastSendTimestampMs= -1.0;

	// Send-rate stats
	ClockTimePoint m_statsWindowStart;
	int m_sentInWindow= 0;
	std::atomic<float> m_messagesPerSecond{0.f};
	std::atomic<int> m_lastPacketCount{0};
	std::atomic<int> m_lastFrameBytes{0};
};
