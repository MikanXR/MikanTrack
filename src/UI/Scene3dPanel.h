#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "glm/ext/matrix_float4x4.hpp"
#include "glm/ext/vector_float2.hpp"
#include "glm/ext/vector_float3.hpp"
#include "glm/ext/vector_int4.hpp"
#include "opencv2/core/mat.hpp"

#include "FaceProfileBinding.h"
#include "AvatarMeshView.h"
#include "AvatarRetarget.h"
#include "FrontCameraRetarget.h"
#include "RetargetStage.h" // RetargetResult
#include "SceneGizmo.h"
#include "SourceSkeleton.h"
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

// The selected chain's handles over the scene, world space, and the one the
// transform gizmo is attached to. The caller fills the inputs each frame and
// drawBody fills the outputs.
struct SceneGizmos
{
	std::vector<SceneGizmoHandle> handles;
	SceneGizmoTarget selected;

	// A click on a handle's marker picked it
	bool bPicked= false;
	SceneGizmoTarget picked;
	// While the selected handle's gizmo is dragged: true, with where it was
	// moved or turned to this frame, and bDragStarted on the drag's first
	// frame
	bool bDragging= false;
	bool bDragStarted= false;
	glm::vec3 draggedPositionWorld{0.f};
	glm::quat draggedFrameWorld{1.f, 0.f, 0.f, 0.f};
	// Escape ended the drag: the caller puts back what the drag changed
	bool bCancelled= false;
};

// What one render of the scene draws. The grid takes the marker axes with
// it. A layer that is on still draws only what the frame holds: the avatar
// needs a loaded model, the front-camera chains a front-camera solve, a gizmo its enable.
// The source pose and the IK goals are off unless a host asks for them: they
// need the result the avatar was posed from, and the IK goals a position
// solve.
struct SceneLayers
{
	bool bGrid= true;
	bool bCameraFrustums= true;
	bool bFusedSkeleton= true;
	bool bPerCameraSkeletons= true;
	bool bAvatar= true;
	bool bFrontCameraSolve= true;
	bool bGizmos= true;
	bool bSourcePose= false;
	bool bIkGoals= false;
};

// Side by side splits a scene's layers between two halves of the view: the
// source half takes what was measured (the source pose, the skeletons and
// the camera frustums), the avatar half what posed the avatar (the avatar,
// the front-camera solve's chains, the gizmos and the IK goals). Both take the
// grid.
void splitSideBySideLayers(const SceneLayers& layers, SceneLayers& outSource, SceneLayers& outAvatar);

// How a host shows the scene beyond its layers
struct SceneViewOptions
{
	// The source half on the left and the avatar half on the right, through
	// the one camera, so they turn together; the overlay while no avatar is
	// shown
	bool bSideBySide= false;
	// A click on the source pose selects the chain under it, read back with
	// consumePickedChain
	bool bPickSourceChains= false;
};

// World is Z-up (marker plane = XY); the renderer and orbit camera are Y-up.
// The one world-to-display conversion: world +Z becomes display +Y, so
// (x, y, z) -> (x, z, -y). Shared with the headless avatar render tool.
const glm::mat4& displayFromWorld();

// A real camera's view of the scene: its undistorted pinhole and its pose
// (OpenCV camera axes to world), for the view's camera choice and the
// avatar capture. In front-camera mode the pose comes from the solve (the camera
// at the measured shoulders), in stereo mode from the extrinsics.
struct SceneCameraMatch
{
	bool valid= false;
	// The configured camera it is, -1 when not told
	int cameraIndex= -1;
	FrontCameraModel camera;
	glm::mat4 cameraToWorld{1.f};
};

// Whether the scene can be looked through a configured camera, and why not
enum class eScenePovAvailability : int
{
	Available,
	// The config has no such camera
	Missing,
	// The camera has no lens calibration
	NotCalibrated,
	// A multi-camera project's camera without extrinsics has no placement
	NoExtrinsics,
	// In a front-camera project only the front camera has a placement, the one
	// the solve gives it
	NotFrontCamera,
};
// The rule over a config alone. A front camera that is available still
// needs a solve with a torso on the frame to have its placement.
eScenePovAvailability getScenePovAvailability(const AppConfig& config, int cameraIndex);

// The view and projection that render the scene the way the camera's
// undistorted frame shows it: a world point lands on the pixel the pinhole
// puts it on, with image row 0 at the top of the rendered image (NDC +Y),
// which is how the panel displays its framebuffer and how the capture
// reads it back
void makeCameraMatchedView(const FrontCameraModel& camera, const glm::mat4& cameraToWorld, float zNear, float zFar,
						   glm::mat4& outView, glm::mat4& outProjection);
// A world point through that view onto the frame's pixels; false behind
// the camera
bool projectCameraMatched(const glm::mat4& viewProjection, const glm::vec3& world, int width, int height,
						  glm::vec2& outPixel);
// A world point through a view-projection and the GL viewport it was drawn
// into (x, y, width, height from the framebuffer's bottom left) onto the
// image item showing that framebuffer flipped, row 0 at the top, whose top
// left is imageMin and whose height is imageHeight; false behind the camera
bool projectToViewImage(const glm::mat4& viewProjection, const glm::ivec4& viewport, const glm::vec2& imageMin,
						float imageHeight, const glm::vec3& world, glm::vec2& outPixel);

// One view onto the scene: the framebuffer it renders into, its orbit
// camera, and where its last draw rendered inside that framebuffer (the
// viewport, the letterbox while the camera view is on) with the
// view-projection it used. The panel holds the Stage View's, and every
// other window that shows the scene holds one of its own.
struct SceneViewState
{
	SceneViewState();
	~SceneViewState();

	std::unique_ptr<GlFrameBuffer> frameBuffer;
	std::unique_ptr<OrbitCamera> camera;
	glm::mat4 viewProjection{1.f};
	// x, y, width, height from the framebuffer's bottom left
	glm::ivec4 viewport{0};
	// Where the source pose was drawn: the same as the view's, or side by
	// side, the source half, while the view's is the avatar half
	glm::mat4 sourceViewProjection{1.f};
	glm::ivec4 sourceViewport{0};
	// Whether the orbit has been aimed at the head yet: the first draw with a
	// head to aim at does it, and the orbit is the user's from then on
	bool bFramedHead= false;
	// The view cube on the last draw: under the mouse, or holding a press
	// that began on it, so the orbit, the picks and the gizmos stand down
	bool bCubeHasMouse= false;
	bool bCubePressed= false;
};

// The orbit angles that look at the target from a named side of the avatar,
// which faces display +X: front, back, left, right, top or bottom, the sides
// being the avatar's own. False for any other name.
bool getSceneViewFaceAngles(const std::string& face, float& outYawDegrees, float& outPitchDegrees);

// The 3D scene: renders the marker-plane grid, marker axes, one frustum per
// calibrated camera, the FUSED hand/arm skeletons (full brightness), each
// camera's unfused skeleton (dimmed, in that camera's color), the avatar,
// the front-camera solve's chains, the gizmos, the source pose the retarget took
// and the IK goals it aimed the avatar's wrists at, each as its layer says, into an
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
	// camera, and runs the orbit, the handle picks and the transform gizmo
	// on the image. The handles are drawn and picked only while their layer
	// is on. Side by side, they belong to the avatar half and the source
	// chains are picked in the source half.
	void drawBody(const TrackingFrameResult& fusedResult,
				  const std::vector<SceneCameraView>& cameras,
				  const std::vector<const TrackingFrameResult*>& perCameraResults,
				  SceneGizmos& gizmos,
				  const SceneLayers& layers,
				  const SceneViewOptions& options= SceneViewOptions());

	// Draws the scene the way drawBody does, through a view the caller
	// holds: its own framebuffer and orbit (dragged and wheeled while the
	// image is hovered), never the camera view and never the gizmos, which
	// belong to the panel's own view. The avatar, the renderers and the
	// frame's inputs are the panel's, shared with every view.
	void drawViewBody(SceneViewState& view, const TrackingFrameResult& fusedResult,
					  const std::vector<SceneCameraView>& cameras,
					  const std::vector<const TrackingFrameResult*>& perCameraResults, const SceneLayers& layers);

	// Renders the scene into target, sized width by height (created or
	// resized as needed), cleared whole and drawn inside viewport (x, y,
	// width, height from the bottom left) with the view-projection given.
	// Leaves target unbound. False without a GL context or a valid size.
	bool renderToTarget(GlFrameBuffer& target, uint16_t width, uint16_t height, const glm::ivec4& viewport,
						const glm::mat4& viewProjection, const SceneLayers& layers,
						const TrackingFrameResult& fusedResult, const std::vector<SceneCameraView>& cameras,
						const std::vector<const TrackingFrameResult*>& perCameraResults, const SceneGizmos& gizmos);

	// One draw of the scene into a part of a target
	struct ScenePass
	{
		glm::ivec4 viewport{0};
		glm::mat4 viewProjection{1.f};
		SceneLayers layers;
	};
	// The same with several passes: the target cleared whole once, then each
	// pass drawn inside its own viewport. The passes' viewports must not
	// overlap, since they share the depth buffer.
	bool renderToTarget(GlFrameBuffer& target, uint16_t width, uint16_t height, const std::vector<ScenePass>& passes,
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

	// The front-camera solver's chains (its world, the camera at the origin) drawn
	// beside the avatar they posed; solveToWorld carries them into the
	// avatar's world. Null draws none.
	void setFrontCameraSolve(const FrontCameraRetarget::SolveState* state, const glm::mat4& solveToWorld);

	// The result the avatar was posed from (copied), for the source pose
	// layer and the IK goals, and where its head is drawn (the source pose
	// carries the head's orientation alone; null draws no head). Null, or a
	// result no pipeline filled, draws neither. A front-camera result's positions
	// are carried into the avatar's world by the front-camera solve's transform,
	// so they draw only while a front-camera solve is set.
	void setSourcePose(const RetargetResult* result, const glm::vec3* headAnchorWorld);
	// The face the front camera's body model tracked on the shown frame
	// (copied), drawn with the source pose's head through the capture's
	// camera; null, or an observation without the face block, draws none
	void setSourceFace(const BodyPoseObservation* body);
	// A front-camera source made by hand has no solve behind it: its positions are
	// the scene's world already, and its palms are given here, each in that
	// world, where a solved source's come from the solve. Null for a source
	// a solve produced.
	void setCustomSourcePalms(const std::array<glm::quat, 2>* palms);
	// The chain highlighted on the source pose
	void setSelectedChain(eRetargetChain chain) { m_selectedChain= chain; }
	// The source skeleton the layer draws and the picking hits
	const SourceSkeleton& getSourceSkeleton();
	// The chain a click on the source pose picked since the last call; false
	// when none was
	bool consumePickedChain(eRetargetChain& outChain);

	// The head an orbit is aimed at (copied): the posed avatar's, or the
	// tracked one while no avatar is loaded. Null leaves every orbit where it
	// is.
	void setFocusHead(const glm::vec3* headWorld);
	// Aims a view's orbit at that head, keeping its angles and its distance.
	// False without a head. Every view does it once on its own, and again on
	// the F key while its image is hovered.
	bool frameHead(SceneViewState& view);
	// The same for the Stage View's own view
	bool frameHead() { return frameHead(m_mainView); }
	// The Stage View's orbit camera
	OrbitCamera& getOrbitCamera() { return *m_mainView.camera; }

	// The view's camera choice, not persisted: -1 is the free (orbit) camera,
	// else the configured camera the view looks through, the frame's aspect
	// letterboxed in the panel while the orbit camera waits untouched.
	// setPovCameras gives the shown frame's cameras each frame, and a chosen
	// camera that is no longer available falls back to the free camera.
	void setPovCameras(const std::vector<eScenePovAvailability>& cameras);
	const std::vector<eScenePovAvailability>& getPovCameras() const { return m_povCameras; }
	int getPovCamera() const { return m_povCamera; }
	// False, leaving the choice as it was, for a camera that is not available
	bool setPovCamera(int cameraIndex);
	// The chosen camera's match on this frame. The view looks through it
	// while it is valid and for the chosen camera, and is the free camera
	// otherwise.
	void setPovMatch(const SceneCameraMatch& match) { m_povMatch= match; }
	// Whether the view looks through a camera on this frame
	bool isPovMatched() const;

	// The camera the capture looks through, apart from the view's choice;
	// an invalid match disables the capture
	void setCaptureMatch(const SceneCameraMatch& match) { m_captureMatch= match; }
	bool hasCaptureMatch() const { return m_captureMatch.valid; }

	// Renders the scene from the capture's camera at the frame's size into a
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
	// Renders the scene through the view at the size of the current
	// window's remaining region, with the view's orbit camera or, while
	// bCameraView, the matched camera letterboxed, and shows it as an image
	// item. False, with a note drawn instead, when the renderer failed.
	// Side by side, the region is split in two, each half framed the same way.
	bool drawViewImage(SceneViewState& view, bool bCameraView, bool bSideBySide, const SceneLayers& layers,
					   const TrackingFrameResult& fusedResult, const std::vector<SceneCameraView>& cameras,
					   const std::vector<const TrackingFrameResult*>& perCameraResults, const SceneGizmos& gizmos,
					   glm::vec2& outImageMin, glm::vec2& outImageSize);
	// Frames a part of the framebuffer: the orbit camera filling it, or the
	// matched camera letterboxed inside it
	void frameRegion(SceneViewState& view, bool bCameraView, const glm::ivec4& region, glm::ivec4& outViewport,
					 glm::mat4& outViewProjection) const;
	// A click on the source pose, told from an orbit drag by how far the
	// mouse moved between the press and the release, picks the chain under
	// the release. A press a gizmo took is not a click.
	void updateSourcePick(const glm::vec2& imageMin, const glm::vec2& imageSize);
	void drawSourceSkeleton();
	void drawIkGoals();
	// The orbit on the image item just drawn, while it is hovered: the left
	// drag turns it (unless bLeftDragOrbits is off, when a gizmo holds the
	// mouse), the middle drag pans, the wheel zooms
	static void updateOrbit(OrbitCamera& camera, bool bLeftDragOrbits);
	// The view cube at the top right of the view's viewport, over the image
	// item just drawn: a drag on it turns the view's orbit about its target
	// and a click on a face, an edge or a corner turns it to look from there.
	// Fills the view's cube state.
	static void drawViewCube(SceneViewState& view, const glm::vec2& imageMin, const glm::vec2& imageSize);
	void drawGizmos(const SceneGizmos& gizmos);
	void drawFrontCameraSolve();
	// Runs the transform gizmo on the selected handle and the click that
	// picks a handle by its marker, over the scene image; returns whether
	// either owns the mouse this frame (the orbit drag and the chain pick
	// stand down). Escape during a drag ends it and reports it cancelled.
	bool updateGizmo(SceneGizmos& gizmos, const glm::vec2& imageMin, const glm::vec2& imageSize);
	// World point -> image pixel; false when behind the camera
	bool projectToImage(const glm::vec3& world, const glm::vec2& imageMin, const glm::vec2& imageSize,
						glm::vec2& outPixel) const;
	void drawSkeleton(const TrackingFrameResult& result, float brightness, const glm::vec3* colorOverride);
	// Draws the avatar when one is loaded and bShow is on; returns whether
	// anything was drawn. An unloaded avatar drops its GPU copy either way.
	bool drawAvatar(const glm::mat4& viewProjection, bool bShow);

	// The Stage View's view, the one the gizmos are picked and drawn in
	SceneViewState m_mainView;
	std::unique_ptr<GlFrameBuffer> m_captureBuffer;
	std::unique_ptr<GlLineRenderer> m_lineRenderer;
	bool m_bLineRendererInitialized= false;
	AvatarMeshView m_meshView;

	std::vector<eScenePovAvailability> m_povCameras;
	int m_povCamera= -1;
	SceneCameraMatch m_povMatch;
	SceneCameraMatch m_captureMatch;
	bool m_bHasFrontCameraSolve= false;
	FrontCameraRetarget::SolveState m_frontCameraSolve;
	glm::mat4 m_solveToWorld{1.f};

	// The source pose layer's inputs, the skeleton built from them (rebuilt
	// when they change) and the selected chain
	bool m_bHasSourcePose= false;
	SourcePose m_sourcePose;
	LossHands m_sourceHands;
	AvatarRetargetGoals m_sourceGoals;
	bool m_bHasHeadAnchor= false;
	glm::vec3 m_headAnchorWorld{0.f};
	bool m_bHasCustomPalms= false;
	std::array<glm::quat, 2> m_customPalms{};
	bool m_bHasSourceFace= false;
	SourceSkeletonInputs::Face m_sourceFace;
	SourceSkeleton m_sourceSkeleton;
	bool m_bSourceSkeletonDirty= true;
	eRetargetChain m_selectedChain= eRetargetChain::LeftArm;
	// A left press on the image that may become a click, where it was, and
	// the chain the last click picked
	bool m_bPickPressArmed= false;
	glm::vec2 m_pickPressPixel{0.f};
	bool m_bHasPickedChain= false;
	eRetargetChain m_pickedChain= eRetargetChain::LeftArm;

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
	// The transform gizmo is mid drag, and the mouse press that a cancelled
	// drag or a handle pick took is still held
	bool m_bGizmoDragging= false;
	bool m_bGizmoPressHeld= false;
	bool m_bGizmoOwnsMouse= false;
	bool m_bShowPerCameraSkeletons= false;
	float m_forearmLengthMeters= 0.25f;
	bool m_bHasFocusHead= false;
	glm::vec3 m_focusHeadWorld{0.f};
};
