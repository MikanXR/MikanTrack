#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "glm/ext/matrix_float4x4.hpp"
#include "glm/ext/vector_float2.hpp"
#include "glm/ext/vector_float3.hpp"
#include "glm/ext/vector_int4.hpp"
#include "opencv2/core/mat.hpp"

#include "FaceProfileBinding.h"
#include "AvatarMeshView.h"
#include "AvatarRetarget.h"
#include "DirectRetarget.h"
#include "TrackingTypes.h"

class AppConfig;
struct AvatarModel;
class AvatarSkeleton;
class GlFrameBuffer;
class GlLineRenderer;
class OrbitCamera;
struct MikanMonoIntrinsics;

// A camera's pose/intrinsics for frustum rendering.
// cameraToWorld maps OpenCV-convention camera space to world space (this is
// the markerFromCamera transform stored in ExtrinsicsConfig).
struct SceneCameraView
{
	glm::mat4 cameraToWorld{1.f};
	bool bHasExtrinsics= false;
	const MikanMonoIntrinsics* intrinsics= nullptr;
};

// One view per configured camera, from the profiles' calibration.
// markerFromCamera maps OpenCV-convention camera space to world; the panel
// applies the GL flip for the frustum itself.
std::vector<SceneCameraView> makeSceneCameraViews(const AppConfig& config);

// Draggable handles over the scene, world space. Per arm: the avatar's elbow
// hint, drawn as a ring joined to the shoulder. The caller fills the inputs
// each frame; draw() fills the outputs.
struct SceneGizmos
{
	struct ElbowHint
	{
		bool bEnabled= false;
		glm::vec3 shoulderWorld{0.f};
		glm::vec3 hintWorld{0.f};

		// While the hint is dragged: true, with where it was dragged to this frame
		bool bDragging= false;
		glm::vec3 draggedHintWorld{0.f};
	};
	std::array<ElbowHint, 2> elbowHints{};
};

// What one render of the scene draws. The grid takes the marker axes with
// it. A layer that is on still draws only what the frame holds: the avatar
// needs a loaded model, the direct chains a direct solve, a gizmo its enable.
struct SceneLayers
{
	bool bGrid= true;
	bool bCameraFrustums= true;
	bool bFusedSkeleton= true;
	bool bPerCameraSkeletons= true;
	bool bAvatar= true;
	bool bDirectSolve= true;
	bool bGizmos= true;
};

// World is Z-up (marker plane = XY); the renderer and orbit camera are Y-up.
// The one world-to-display conversion: world +Z becomes display +Y, so
// (x, y, z) -> (x, z, -y). Shared with the headless avatar render tool.
const glm::mat4& displayFromWorld();

// A real camera's view of the scene: its undistorted pinhole and its pose
// (OpenCV camera axes to world), for the view-from-camera toggle and the
// avatar capture. In direct mode the pose comes from the solve (the camera
// at the measured shoulders), in stereo mode from the extrinsics.
struct SceneCameraMatch
{
	bool valid= false;
	DirectCameraModel camera;
	glm::mat4 cameraToWorld{1.f};
};

// The view and projection that render the scene the way the camera's
// undistorted frame shows it: a world point lands on the pixel the pinhole
// puts it on, with image row 0 at the top of the rendered image (NDC +Y),
// which is how the panel displays its framebuffer and how the capture
// reads it back
void makeCameraMatchedView(const DirectCameraModel& camera, const glm::mat4& cameraToWorld, float zNear, float zFar,
						   glm::mat4& outView, glm::mat4& outProjection);
// A world point through that view onto the frame's pixels; false behind
// the camera
bool projectCameraMatched(const glm::mat4& viewProjection, const glm::vec3& world, int width, int height,
						  glm::vec2& outPixel);

// The 3D scene: renders the marker-plane grid, marker axes, one frustum per
// calibrated camera, the FUSED hand/arm skeletons (full brightness), each
// camera's unfused skeleton (dimmed, in that camera's color), the avatar,
// the direct solve's chains and the gizmos, each as its layer says, into an
// FBO shown as an ImGui image. It owns no window: a host draws the body.
//
// World convention (from calibration): right-handed, meters, origin at the
// marker center, +Z out of the table. For display this is rotated into the
// renderer's Y-up space (world +Z becomes view up).
class Scene3dPanel
{
public:
	Scene3dPanel();
	~Scene3dPanel();

	// Draws the scene into the current window's remaining content region,
	// with the orbit camera or, while the camera view is on, the matched
	// camera, and runs the orbit and the gizmo drags on the image. The
	// gizmos are picked only while their layer is on.
	void drawBody(const TrackingFrameResult& fusedResult,
				  const std::vector<SceneCameraView>& cameras,
				  const std::vector<const TrackingFrameResult*>& perCameraResults,
				  SceneGizmos& gizmos,
				  const SceneLayers& layers);

	// Renders the scene into target, sized width by height (created or
	// resized as needed), cleared whole and drawn inside viewport (x, y,
	// width, height from the bottom left) with the view-projection given.
	// Leaves target unbound. False without a GL context or a valid size.
	bool renderToTarget(GlFrameBuffer& target, uint16_t width, uint16_t height, const glm::ivec4& viewport,
						const glm::mat4& viewProjection, const SceneLayers& layers,
						const TrackingFrameResult& fusedResult, const std::vector<SceneCameraView>& cameras,
						const std::vector<const TrackingFrameResult*>& perCameraResults, const SceneGizmos& gizmos);

	// Forearm length used to place the elbow estimate (meters); the
	// direction is measured by the wrist IMU, only the length is assumed
	void setForearmLength(float meters) { m_forearmLengthMeters= meters; }

	// The avatar to draw (null = none). The generation counter tells the panel
	// when the model changed and its GPU copy must be rebuilt.
	void setAvatar(std::shared_ptr<const AvatarModel> model, std::shared_ptr<const AvatarSkeleton> skeleton,
				   uint32_t generation);
	// The user's show-in-scene setting. A host's avatar layer follows it, and
	// the capture draws the avatar only while it is on.
	bool getShowAvatar() const { return m_bShowAvatar; }
	void setShowAvatar(bool bShow) { m_bShowAvatar= bShow; }
	// The retargeted pose to draw (copied); null draws the rest pose at the
	// world origin
	void setAvatarPose(const AvatarPose* pose);
	// The face to show on the avatar (copied); null rests the face. The
	// avatar's morph targets take the frame's evaluated profile values
	// through the binding of the same generation, the way a VMC receiver
	// holding the same file would. A frame from another generation shows the
	// rest face.
	void setAvatarFace(const FaceSources* face);
	// The loaded avatar's face profile binding
	void setFaceBinding(std::shared_ptr<const FaceProfileBinding> binding) { m_faceBinding= std::move(binding); }

	// The Fusion stage's per-camera skeletons toggle. A host's per-camera
	// layer follows it, and the capture draws them only while it is on.
	bool getShowPerCameraSkeletons() const { return m_bShowPerCameraSkeletons; }
	void setShowPerCameraSkeletons(bool bShow) { m_bShowPerCameraSkeletons= bShow; }

	// The direct solver's chains (its world, the camera at the origin) drawn
	// beside the avatar they posed; solveToWorld carries them into the
	// avatar's world. Null draws none.
	void setDirectSolve(const DirectRetarget::SolveState* state, const glm::mat4& solveToWorld);

	// The camera the view-from-camera toggle and the capture look through;
	// an invalid match disables both
	void setCameraMatch(const SceneCameraMatch& match) { m_cameraMatch= match; }
	bool hasCameraMatch() const { return m_cameraMatch.valid; }
	// On, the panel shows the scene from the matched camera, the frame's
	// aspect letterboxed in the panel; the orbit camera waits untouched
	bool getViewFromCamera() const { return m_bViewFromCamera; }
	void setViewFromCamera(bool bOn) { m_bViewFromCamera= bOn; }

	// Renders the scene from the matched camera at the frame's size into a
	// BGR image, the way the dump's annotated frame is laid out, so the two
	// compare pixel for pixel. Every layer but the gizmos, the avatar and the
	// per-camera skeletons as their settings say. False without a match or a
	// GL context. Needs a current GL context, not an ImGui frame.
	bool captureFromCamera(const TrackingFrameResult& fusedResult, const std::vector<SceneCameraView>& cameras,
						   const std::vector<const TrackingFrameResult*>& perCameraResults, cv::Mat& outBgr);

private:
	// Draws the layers that are on into the bound framebuffer's current
	// viewport with the view and projection given
	void renderScene(const TrackingFrameResult& fusedResult, const std::vector<SceneCameraView>& cameras,
					 const std::vector<const TrackingFrameResult*>& perCameraResults, const SceneGizmos& gizmos,
					 const glm::mat4& viewProjection, const SceneLayers& layers);
	bool startRenderers();
	void drawGizmos(const SceneGizmos& gizmos);
	void drawDirectSolve();
	// Picks and drags the gizmos with the mouse over the scene image; returns
	// whether a gizmo owns the mouse this frame (the orbit drag stands down)
	bool updateGizmoDrag(SceneGizmos& gizmos, const glm::vec2& imageMin, const glm::vec2& imageSize);
	// World point -> image pixel; false when behind the camera
	bool projectToImage(const glm::vec3& world, const glm::vec2& imageMin, const glm::vec2& imageSize,
						glm::vec2& outPixel) const;
	void drawSkeleton(const TrackingFrameResult& result, float brightness, const glm::vec3* colorOverride);
	// Draws the avatar when one is loaded and bShow is on; returns whether
	// anything was drawn. An unloaded avatar drops its GPU copy either way.
	bool drawAvatar(const glm::mat4& viewProjection, bool bShow);

	std::unique_ptr<GlFrameBuffer> m_frameBuffer;
	std::unique_ptr<GlFrameBuffer> m_captureBuffer;
	std::unique_ptr<GlLineRenderer> m_lineRenderer;
	std::unique_ptr<OrbitCamera> m_camera;
	bool m_bLineRendererInitialized= false;
	AvatarMeshView m_meshView;

	SceneCameraMatch m_cameraMatch;
	bool m_bViewFromCamera= false;
	// The last draw's view and where it rendered inside the panel's
	// framebuffer (the letterbox when the camera view is on), for the gizmo
	// picking
	glm::mat4 m_viewProjection{1.f};
	int m_viewport[4]= {0, 0, 0, 0};
	bool m_bHasDirectSolve= false;
	DirectRetarget::SolveState m_directSolve;
	glm::mat4 m_solveToWorld{1.f};

	std::shared_ptr<const AvatarModel> m_avatarModel;
	std::shared_ptr<const AvatarSkeleton> m_avatarSkeleton;
	bool m_bShowAvatar= true;
	bool m_bHasAvatarPose= false;
	AvatarPose m_avatarPose;
	std::shared_ptr<const FaceProfileBinding> m_faceBinding;
	bool m_bHasAvatarFace= false;
	FaceSources m_avatarFace;
	std::vector<std::vector<float>> m_morphWeights;
	std::vector<glm::mat4> m_posedGlobals;
	// The gizmo being dragged (-1 none) and the camera-facing plane it moves
	// in, display space
	int m_draggedElbowHint= -1;
	glm::vec3 m_dragPlanePoint{0.f};
	glm::vec3 m_dragPlaneNormal{0.f};
	bool m_bShowPerCameraSkeletons= false;
	float m_forearmLengthMeters= 0.25f;
};
