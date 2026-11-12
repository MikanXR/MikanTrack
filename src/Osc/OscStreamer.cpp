#include "OscStreamer.h"

#include <algorithm>

#include "Logger.h"
#include "TrackingTypes.h"

// VMC protocol addresses (https://protocol.vmc.info)
static const char* k_vmcOkAddress= "/VMC/Ext/OK";
static const char* k_vmcTimeAddress= "/VMC/Ext/T";
static const char* k_vmcRootAddress= "/VMC/Ext/Root/Pos";
static const char* k_vmcBoneAddress= "/VMC/Ext/Bone/Pos";
static const char* k_vmcBlendValueAddress= "/VMC/Ext/Blend/Val";
static const char* k_vmcBlendApplyAddress= "/VMC/Ext/Blend/Apply";
static const char* k_vmcAvatarAddress= "/VMC/Ext/VRM";

static void addVec3(OscMessage& message, const glm::vec3& point)
{
	message.addFloat(point.x).addFloat(point.y).addFloat(point.z);
}

static void addQuat(OscMessage& message, const glm::quat& rotation)
{
	message.addFloat(rotation.x).addFloat(rotation.y).addFloat(rotation.z).addFloat(rotation.w);
}

OscStreamer::~OscStreamer()
{
	shutdown();
}

bool OscStreamer::startup()
{
	std::lock_guard<std::mutex> lock(m_mutex);

	if (m_isRunning)
		return true;

	if (!m_socket.open())
	{
		MIKAN_LOG_ERROR("OscStreamer::startup") << "Failed to open UDP socket";
		return false;
	}

	m_isRunning= true;
	m_lastSendTimestampMs= -1.0;
	m_sentInWindow= 0;
	m_statsWindowStart= std::chrono::steady_clock::now();
	m_startTime= m_statsWindowStart;
	m_messagesPerSecond.store(0.f, std::memory_order_relaxed);
	m_lastPacketCount.store(0, std::memory_order_relaxed);
	m_lastFrameBytes.store(0, std::memory_order_relaxed);
	m_bFaceWasPresent= false;
	m_sentBlendNames.clear();
	m_hasSentAvatar= false;

	// The target is logged by setConfig instead: startup runs before the app's
	// config reaches the streamer, so anything named here would be a default
	MIKAN_LOG_INFO("OscStreamer::startup") << "UDP socket open";

	return true;
}

void OscStreamer::shutdown()
{
	std::lock_guard<std::mutex> lock(m_mutex);

	if (m_isRunning)
	{
		m_socket.close();
		m_isRunning= false;
		m_messagesPerSecond.store(0.f, std::memory_order_relaxed);
		m_lastPacketCount.store(0, std::memory_order_relaxed);
		m_lastFrameBytes.store(0, std::memory_order_relaxed);
	}
}

OscStreamerConfig OscStreamer::getConfig() const
{
	std::lock_guard<std::mutex> lock(m_mutex);
	return m_config;
}

void OscStreamer::setConfig(const OscStreamerConfig& config)
{
	std::lock_guard<std::mutex> lock(m_mutex);

	const bool bTargetChanged= config.targetIp != m_config.targetIp || config.port != m_config.port;

	// Announce at once to a new target or for a new avatar, rather than at
	// the next one-second repeat. A face profile edit swaps only the binding
	// and leaves the repeat alone.
	const bool bAvatarChanged= config.avatarPath != m_config.avatarPath || config.avatarTitle != m_config.avatarTitle ||
							   config.avatarSha256 != m_config.avatarSha256;
	m_config= config;
	if (bAvatarChanged || bTargetChanged)
		m_hasSentAvatar= false;

	if (bTargetChanged)
	{
		MIKAN_LOG_INFO("OscStreamer::setConfig")
			<< "Streaming VMC to " << m_config.targetIp << ":" << m_config.port;
	}
}

void OscStreamer::sendFrame(const TrackingFrameResult& frame, const RetargetResult& retarget)
{
	std::lock_guard<std::mutex> lock(m_mutex);

	if (!m_isRunning || !m_config.enabled || !m_socket.isOpen())
		return;

	// Rate decimation on frame timestamps: skip the frame if it arrived less
	// than one send interval after the last sent frame. A backwards timestamp
	// jump (video restart) resets the gate.
	if (m_config.maxRateHz > 0.f && m_lastSendTimestampMs >= 0.0 && frame.timestampMs >= m_lastSendTimestampMs)
	{
		const double minIntervalMs= 1000.0 / static_cast<double>(m_config.maxRateHz);
		if ((frame.timestampMs - m_lastSendTimestampMs) < minIntervalMs)
			return;
	}
	m_lastSendTimestampMs= frame.timestampMs;

	const ClockTimePoint now= std::chrono::steady_clock::now();

	encodeFrameLocked(frame, retarget, now, m_scratchPackets);

	// One frame can be several datagrams. The stats count FRAMES, so the rate
	// readout does not depend on how the frame was split.
	bool bSentAny= false;
	int frameBytes= 0;
	for (const std::vector<uint8_t>& packet : m_scratchPackets)
	{
		bSentAny|= m_socket.sendTo(m_config.targetIp, m_config.port,
								   packet.data(), static_cast<int>(packet.size()));
		frameBytes+= static_cast<int>(packet.size());
	}
	if (bSentAny)
		m_sentInWindow++;
	m_lastPacketCount.store(static_cast<int>(m_scratchPackets.size()), std::memory_order_relaxed);
	m_lastFrameBytes.store(frameBytes, std::memory_order_relaxed);

	updateSendStats(now);
}

void OscStreamer::encodeFrame(const TrackingFrameResult& frame, const RetargetResult& retarget,
							  std::vector<std::vector<uint8_t>>& outPackets)
{
	std::lock_guard<std::mutex> lock(m_mutex);
	encodeFrameLocked(frame, retarget, std::chrono::steady_clock::now(), outPackets);
}

void OscStreamer::encodeFrameLocked(const TrackingFrameResult& frame, const RetargetResult& retarget,
									const ClockTimePoint& now, std::vector<std::vector<uint8_t>>& outPackets)
{
	m_bundle.clear();
	m_bundle.setTimeTag(k_oscTimeTagImmediate);

	appendVmcMessages(frame, retarget, now);

	// Pack the frame's messages into as few complete bundles as fit. Splitting
	// here rather than letting IP fragment the datagram matters because a
	// receiver drops a whole bundle for one lost fragment, and because the OSC
	// library several VMC tools are built on defaults to a 2048-byte receive
	// buffer - a single 3.4 KB bundle simply never arrives.
	size_t packetCount= 0;
	// Reuses the outer vector's buffers across frames (the packet count is
	// stable), so this stays allocation-light like the rest of the encode path
	auto emitPacket= [&](size_t firstMessage, size_t count) {
		if (packetCount == outPackets.size())
			outPackets.emplace_back();
		else
			outPackets[packetCount].clear();

		m_bundle.encodeRange(firstMessage, count, outPackets[packetCount]);
		packetCount++;
	};

	const size_t messageCount= m_bundle.getMessageCount();
	size_t firstInPacket= 0;
	size_t packetBytes= OscBundle::k_headerSize;

	for (size_t messageIndex= 0; messageIndex < messageCount; ++messageIndex)
	{
		const size_t messageBytes= m_bundle.getMessageEncodedSize(messageIndex);

		// Flush before adding, so the packet under construction stays legal.
		// A single message over the limit still goes out alone: dropping it
		// would silently lose a bone, and no message this streams comes close.
		if (messageIndex > firstInPacket && packetBytes + messageBytes > k_maxDatagramBytes)
		{
			emitPacket(firstInPacket, messageIndex - firstInPacket);
			firstInPacket= messageIndex;
			packetBytes= OscBundle::k_headerSize;
		}

		packetBytes+= messageBytes;
	}

	if (messageCount > firstInPacket)
		emitPacket(firstInPacket, messageCount - firstInPacket);

	outPackets.resize(packetCount);
}

void OscStreamer::appendVmcMessages(const TrackingFrameResult& frame, const RetargetResult& retarget,
									const ClockTimePoint& now)
{
	// The retarget stage has already gated, held, frozen and rested what it
	// hands on, so a lost hand reaches the wire the way the stage decided:
	// as a held or frozen arm, or as bones that stop arriving
	bool bTracking= false;
	if (retarget.pose.valid)
	{
		if (m_config.avatarSkeleton != nullptr)
			VmcRetarget::buildPoseFromAvatar(retarget.pose, *m_config.avatarSkeleton, m_vmcPose);
		else
			m_vmcPose.clear();

		bTracking= retarget.pose.bones[(int)eHumanoidBone::LeftUpperArm].present ||
				   retarget.pose.bones[(int)eHumanoidBone::RightUpperArm].present;
	}
	else
	{
		VmcRetarget::VmcBodyLengths lengths;
		lengths.shoulderWidthMeters= m_config.shoulderWidthMeters;
		lengths.upperArmLengthMeters= m_config.upperArmLengthMeters;
		lengths.forearmLengthMeters= m_config.forearmLengthMeters;
		lengths.headOffsetMeters= m_config.vmcHeadOffsetMeters;

		const bool bSideCarried[2]= {retarget.hands.bCarried[0], retarget.hands.bCarried[1]};
		VmcRetarget::buildPose(retarget.hands.poses, bSideCarried, frame.head, lengths, m_vmcPose);

		bTracking= bSideCarried[0] || bSideCarried[1];
	}

	// /VMC/Ext/OK ,iiii -- loaded, calibration state, calibration mode,
	// tracking status. Calibration always reads as done in normal mode: this
	// rig calibrates itself against a printed board long before it streams, so
	// there is no receiver-driven calibration step for anyone to wait on.
	{
		OscMessage& okMessage= m_bundle.addMessage(k_vmcOkAddress);
		okMessage.addInt32(1).addInt32(3).addInt32(0).addInt32(bTracking ? 1 : 0);
	}

	// /VMC/Ext/T ,f -- the sender's own clock, which a receiver uses to tell a
	// live stream from a stalled one
	{
		const std::chrono::duration<float> elapsed= now - m_startTime;
		m_bundle.addMessage(k_vmcTimeAddress).addFloat(elapsed.count());
	}

	// /VMC/Ext/Root/Pos ,sfffffff -- identity on purpose. This is an upper-body
	// tracker anchored to a desk marker, so the marker frame is not a place to
	// put an avatar; the receiver keeps whatever root it already has.
	{
		OscMessage& rootMessage= m_bundle.addMessage(k_vmcRootAddress);
		rootMessage.addString("root");
		addVec3(rootMessage, glm::vec3(0.f));
		addQuat(rootMessage, glm::quat(1.f, 0.f, 0.f, 0.f));
	}

	// /VMC/Ext/Bone/Pos ,sfffffff -- one per MEASURED bone. A bone left out
	// stays at the avatar's rest pose on the receiving side, which is exactly
	// the reference the streamed rotations are relative to.
	for (int boneIndex= 0; boneIndex < VmcRetarget::VMC_BONE_COUNT; ++boneIndex)
	{
		const VmcRetarget::VmcBone& bone= m_vmcPose.bones[boneIndex];
		if (!bone.present)
			continue;

		OscMessage& boneMessage= m_bundle.addMessage(k_vmcBoneAddress);
		boneMessage.addString(VmcRetarget::boneName((VmcRetarget::eVmcBone)boneIndex));
		addVec3(boneMessage, bone.localPosition);
		addQuat(boneMessage, bone.localRotation);
	}

	appendVmcBlendMessages(frame);
	appendVmcAvatarMessage(now);
}

void OscStreamer::appendVmcBlendMessages(const TrackingFrameResult& frame)
{
	const bool bFacePresent= frame.face.isPresent();
	if (!bFacePresent && !m_bFaceWasPresent)
		return;
	const FaceProfileBinding* binding= m_config.faceBinding.get();
	if (binding == nullptr)
		return;
	// Values evaluated under another binding name other targets; the next
	// frame carries this binding's
	if (bFacePresent && frame.face.outputs.generation != binding->getGeneration())
		return;
	m_bFaceWasPresent= bFacePresent;

	// A stopped face sends zeros once: a receiver holds the last value of a
	// blendshape that stops arriving, which would freeze a blink mid-closure
	const std::vector<std::string>& names= binding->getWireNames();
	if (bFacePresent)
		binding->combineWire(frame.face.outputs.targetValues, m_blendValues);
	else
		m_blendValues.assign(names.size(), 0.f);

	for (size_t nameIndex= 0; nameIndex < names.size(); ++nameIndex)
	{
		OscMessage& blendMessage= m_bundle.addMessage(k_vmcBlendValueAddress);
		blendMessage.addString(names[nameIndex].c_str()).addFloat(m_blendValues[nameIndex]);
	}
	// The names a previous binding sent and this one does not relax too
	for (const std::string& sentName : m_sentBlendNames)
	{
		if (std::find(names.begin(), names.end(), sentName) == names.end())
		{
			OscMessage& blendMessage= m_bundle.addMessage(k_vmcBlendValueAddress);
			blendMessage.addString(sentName.c_str()).addFloat(0.f);
		}
	}
	m_bundle.addMessage(k_vmcBlendApplyAddress);
	if (m_sentBlendNames != names)
		m_sentBlendNames= names;
}

void OscStreamer::appendVmcAvatarMessage(const ClockTimePoint& now)
{
	if (m_config.avatarPath.empty())
		return;
	if (m_hasSentAvatar && now - m_lastAvatarTime < std::chrono::seconds(1))
		return;
	m_hasSentAvatar= true;
	m_lastAvatarTime= now;

	OscMessage& avatarMessage= m_bundle.addMessage(k_vmcAvatarAddress);
	avatarMessage.addString(m_config.avatarPath.c_str())
		.addString(m_config.avatarTitle.c_str())
		.addString(m_config.avatarSha256.c_str());
}

void OscStreamer::updateSendStats(const ClockTimePoint& now)
{
	const std::chrono::duration<float> windowElapsed= now - m_statsWindowStart;
	if (windowElapsed >= std::chrono::seconds(1))
	{
		m_messagesPerSecond.store(
			static_cast<float>(m_sentInWindow) / windowElapsed.count(),
			std::memory_order_relaxed);
		m_sentInWindow= 0;
		m_statsWindowStart= now;
	}
}
