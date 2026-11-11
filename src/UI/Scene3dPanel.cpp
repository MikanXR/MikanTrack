#include "Scene3dPanel.h"

#include "GL/glew.h"

#include "imgui.h"

#include "LocText.h"

#include "glm/ext/matrix_transform.hpp"
#include "glm/gtc/constants.hpp"

#include "AppConfig.h"
#include "AvatarSkeleton.h"
#include "Colors.h"
#include "DebugDraw.h"
#include "GlFrameBuffer.h"
#include "GlLineRenderer.h"
#include "HandPoseModel.h"
#include "MikanVideoSourceTypes.h"
#include "OrbitCamera.h"

#include "glm/gtc/quaternion.hpp"
#include "opencv2/imgproc.hpp"

static const glm::mat4 k_displayFromWorld=
	glm::rotate(glm::mat4(1.f), -glm::half_pi<float>(), glm::vec3(1.f, 0.f, 0.f));

const glm::mat4& displayFromWorld()
{
	return k_displayFromWorld;
}

// OpenCV camera convention (+Y down, +Z forward) <-> GL camera convention
// (+Y up, -Z forward): 180-degree rotation about X. Its own inverse.
static const glm::mat4 k_glFromCvFlip(
	glm::vec4(1, 0, 0, 0),
	glm::vec4(0, -1, 0, 0),
	glm::vec4(0, 0, -1, 0),
	glm::vec4(0, 0, 0, 1));

void makeCameraMatchedView(const DirectCameraModel& camera, const glm::mat4& cameraToWorld, float zNear, float zFar,
						   glm::mat4& outView, glm::mat4& outProjection)
{
	// The camera's frame in display space: its CV axes to world, the world to
	// display, and the CV-to-GL flip so the GL camera looks down -Z
	outView= glm::inverse(k_displayFromWorld * cameraToWorld * k_glFromCvFlip);

	// The off-center pinhole: for a CV camera point (x, y, z) the pixel is
	// (fx x / z + cx, fy y / z + cy), and NDC is (2 u / w - 1, 1 - 2 v / h).
	// In GL camera space that point is (x, -y, -z) with clip w = z, which
	// gives these entries (glm indexes [column][row]).
	const float w= (float)camera.width;
	const float h= (float)camera.height;
	outProjection= glm::mat4(0.f);
	outProjection[0][0]= 2.f * camera.fx / w;
	outProjection[1][1]= 2.f * camera.fy / h;
	outProjection[2][0]= 1.f - 2.f * camera.cx / w;
	outProjection[2][1]= 2.f * camera.cy / h - 1.f;
	outProjection[2][2]= -(zFar + zNear) / (zFar - zNear);
	outProjection[2][3]= -1.f;
	outProjection[3][2]= -2.f * zFar * zNear / (zFar - zNear);
}

bool projectCameraMatched(const glm::mat4& viewProjection, const glm::vec3& world, int width, int height,
						  glm::vec2& outPixel)
{
	const glm::vec4 clip= viewProjection * k_displayFromWorld * glm::vec4(world, 1.f);
	if (clip.w <= 1e-6f)
		return false;
	const glm::vec2 ndc= glm::vec2(clip) / clip.w;
	outPixel= glm::vec2((ndc.x * 0.5f + 0.5f) * (float)width, (0.5f - ndc.y * 0.5f) * (float)height);
	return true;
}

bool projectToViewImage(const glm::mat4& viewProjection, const glm::ivec4& viewport, const glm::vec2& imageMin,
						float imageHeight, const glm::vec3& world, glm::vec2& outPixel)
{
	const glm::vec4 clip= viewProjection * k_displayFromWorld * glm::vec4(world, 1.f);
	if (clip.w <= 1e-6f)
		return false;
	const glm::vec2 ndc= glm::vec2(clip) / clip.w;
	// The image shows the framebuffer flipped, so NDC +Y is the image top,
	// and the viewport sits inside it from the bottom up
	const float viewportTop= imageHeight - (float)(viewport.y + viewport.w);
	outPixel= imageMin +
		glm::vec2((float)viewport.x + (ndc.x * 0.5f + 0.5f) * (float)viewport.z,
				  viewportTop + (0.5f - ndc.y * 0.5f) * (float)viewport.w);
	return true;
}

void splitSideBySideLayers(const SceneLayers& layers, SceneLayers& outSource, SceneLayers& outAvatar)
{
	outSource= layers;
	outSource.bAvatar= false;
	outSource.bDirectSolve= false;
	outSource.bGizmos= false;
	outSource.bIkGoals= false;

	outAvatar= layers;
	outAvatar.bCameraFrustums= false;
	outAvatar.bFusedSkeleton= false;
	outAvatar.bPerCameraSkeletons= false;
	outAvatar.bSourcePose= false;
}

// The pick: how near a click lands to a bone or a joint, and how far the
// mouse may move between the press and the release for it to be a click
// rather than the start of an orbit drag (below ImGui's drag threshold, at
// which the orbit starts turning)
static constexpr float kSourcePickTolerancePixels= 6.f;
static constexpr float kClickSlopPixels= 4.f;
// The selected chain's halo: a ring of lines this far around each bone,
// meters, and its color
static constexpr float kHaloRadius= 0.004f;
static const glm::vec3 k_haloColor(0.45f, 0.75f, 1.f);
// The IK goal's marker: a cross of this half size, meters
static constexpr float kIkGoalHalfSize= 0.025f;
static const glm::vec3 k_ikGoalColor(1.f, 0.3f, 0.85f);

// Distinct tint per camera for frustums + dimmed per-camera skeletons
static const glm::vec3 k_cameraColors[4]= {
	glm::vec3(1.f, 0.9f, 0.3f),  // yellow
	glm::vec3(0.4f, 0.9f, 1.f),  // cyan
	glm::vec3(1.f, 0.5f, 0.9f),  // magenta
	glm::vec3(0.6f, 1.f, 0.5f),  // green
};

SceneViewState::SceneViewState()
	: frameBuffer(std::make_unique<GlFrameBuffer>())
	, camera(std::make_unique<OrbitCamera>())
{
	camera->setOrbitLocation(30.f, -40.f, 1.8f);
}

SceneViewState::~SceneViewState()= default;

Scene3dPanel::Scene3dPanel()
	: m_captureBuffer(std::make_unique<GlFrameBuffer>())
	, m_lineRenderer(std::make_unique<GlLineRenderer>())
{
}

void Scene3dPanel::setAvatar(std::shared_ptr<const AvatarModel> model, std::shared_ptr<const AvatarSkeleton> skeleton,
							 uint32_t generation)
{
	m_avatarModel= std::move(model);
	m_avatarSkeleton= std::move(skeleton);
	m_meshView.setAvatar(m_avatarModel, generation);
}

void Scene3dPanel::setAvatarFace(const FaceSources* face)
{
	m_bHasAvatarFace= face != nullptr && face->isPresent();
	if (m_bHasAvatarFace)
		m_avatarFace= *face;
}

void Scene3dPanel::setAvatarPose(const AvatarPose* pose)
{
	m_bHasAvatarPose= pose != nullptr && pose->valid;
	if (m_bHasAvatarPose)
		m_avatarPose= *pose;
}

void Scene3dPanel::setDirectSolve(const DirectRetarget::SolveState* state, const glm::mat4& solveToWorld)
{
	m_bHasDirectSolve= state != nullptr && state->torsoValid;
	if (m_bHasDirectSolve)
	{
		m_directSolve= *state;
		m_solveToWorld= solveToWorld;
	}
	// The direct source pose is placed by the same transform
	m_bSourceSkeletonDirty= true;
}

void Scene3dPanel::setSourcePose(const RetargetResult* result, const glm::vec3* headAnchorWorld)
{
	m_bHasSourcePose= result != nullptr && result->source.pipeline != SourcePose::ePipeline::None;
	if (m_bHasSourcePose)
	{
		m_sourcePose= result->source;
		m_sourceHands= result->hands;
		m_sourceGoals= result->goals;
	}
	m_bHasHeadAnchor= headAnchorWorld != nullptr;
	if (m_bHasHeadAnchor)
		m_headAnchorWorld= *headAnchorWorld;
	m_bSourceSkeletonDirty= true;
}

const SourceSkeleton& Scene3dPanel::getSourceSkeleton()
{
	if (m_bSourceSkeletonDirty)
	{
		m_bSourceSkeletonDirty= false;
		SourceSkeletonInputs inputs;
		if (m_bHasSourcePose)
		{
			inputs.source= &m_sourcePose;
			inputs.hands= &m_sourceHands;
			inputs.directToWorld= m_bHasDirectSolve ? &m_solveToWorld : nullptr;
			inputs.headAnchorWorld= m_bHasHeadAnchor ? &m_headAnchorWorld : nullptr;
		}
		buildSourceSkeleton(inputs, m_sourceSkeleton);
	}
	return m_sourceSkeleton;
}

bool Scene3dPanel::consumePickedChain(eRetargetChain& outChain)
{
	if (!m_bHasPickedChain)
		return false;
	m_bHasPickedChain= false;
	outChain= m_pickedChain;
	return true;
}

Scene3dPanel::~Scene3dPanel()= default;

bool Scene3dPanel::startRenderers()
{
	if (!m_bLineRendererInitialized)
		m_bLineRendererInitialized= m_lineRenderer->startup();
	return m_bLineRendererInitialized;
}

bool Scene3dPanel::renderToTarget(GlFrameBuffer& target, uint16_t width, uint16_t height,
								  const glm::ivec4& viewport, const glm::mat4& viewProjection,
								  const SceneLayers& layers, const TrackingFrameResult& fusedResult,
								  const std::vector<SceneCameraView>& cameras,
								  const std::vector<const TrackingFrameResult*>& perCameraResults,
								  const SceneGizmos& gizmos)
{
	ScenePass pass;
	pass.viewport= viewport;
	pass.viewProjection= viewProjection;
	pass.layers= layers;
	return renderToTarget(target, width, height, std::vector<ScenePass>{pass}, fusedResult, cameras, perCameraResults,
						  gizmos);
}

bool Scene3dPanel::renderToTarget(GlFrameBuffer& target, uint16_t width, uint16_t height,
								  const std::vector<ScenePass>& passes, const TrackingFrameResult& fusedResult,
								  const std::vector<SceneCameraView>& cameras,
								  const std::vector<const TrackingFrameResult*>& perCameraResults,
								  const SceneGizmos& gizmos)
{
	if (width == 0 || height == 0 || !startRenderers())
		return false;
	if (!target.getIsValid() ? !target.init(width, height) : !target.resize(width, height))
		return false;

	target.bindFrameBuffer();
	glClearColor(0.05f, 0.05f, 0.07f, 1.f);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	for (const ScenePass& pass : passes)
	{
		glViewport(pass.viewport.x, pass.viewport.y, pass.viewport.z, pass.viewport.w);
		renderScene(fusedResult, cameras, perCameraResults, gizmos, pass.viewProjection, pass.layers);
	}
	target.unbindFrameBuffer();
	return true;
}

void Scene3dPanel::drawBody(const TrackingFrameResult& fusedResult, const std::vector<SceneCameraView>& cameras,
							const std::vector<const TrackingFrameResult*>& perCameraResults, SceneGizmos& gizmos,
							const SceneLayers& layers, const SceneViewOptions& options)
{
	for (SceneGizmos::ElbowHint& hint : gizmos.elbowHints)
		hint.bDragging= false;

	const bool bCameraView= m_bViewFromCamera && m_cameraMatch.valid && m_cameraMatch.camera.width > 0 &&
		m_cameraMatch.camera.height > 0;
	// Side by side needs an avatar for its second half
	const bool bSideBySide= options.bSideBySide && layers.bAvatar && m_avatarModel != nullptr;
	glm::vec2 imageMin, imageSize;
	if (!drawViewImage(m_mainView, bCameraView, bSideBySide, layers, fusedResult, cameras, perCameraResults, gizmos,
					   imageMin, imageSize))
	{
		m_bPickPressArmed= false;
		return;
	}

	// The gizmos and the orbit belong to the orbit view: the camera view is
	// a fixed viewpoint
	bool bGizmoOwnsMouse= false;
	if (bCameraView || !layers.bGizmos)
		m_draggedElbowHint= -1;
	else
		bGizmoOwnsMouse= updateGizmoDrag(gizmos, imageMin, imageSize);
	// After the gizmos, so a press a gizmo took is not a click
	if (options.bPickSourceChains && layers.bSourcePose)
		updateSourcePick(imageMin, imageSize);
	else
		m_bPickPressArmed= false;
	if (!bCameraView)
		updateOrbit(*m_mainView.camera, !bGizmoOwnsMouse);
}

void Scene3dPanel::drawViewBody(SceneViewState& view, const TrackingFrameResult& fusedResult,
								const std::vector<SceneCameraView>& cameras,
								const std::vector<const TrackingFrameResult*>& perCameraResults,
								const SceneLayers& layers)
{
	SceneLayers viewLayers= layers;
	viewLayers.bGizmos= false;
	const SceneGizmos noGizmos;
	glm::vec2 imageMin, imageSize;
	if (drawViewImage(view, false, false, viewLayers, fusedResult, cameras, perCameraResults, noGizmos, imageMin,
					  imageSize))
		updateOrbit(*view.camera, true);
}

void Scene3dPanel::frameRegion(SceneViewState& view, bool bCameraView, const glm::ivec4& region,
							   glm::ivec4& outViewport, glm::mat4& outViewProjection) const
{
	if (bCameraView)
	{
		const float frameAspect= (float)m_cameraMatch.camera.width / (float)m_cameraMatch.camera.height;
		int viewWidth= region.z;
		int viewHeight= (int)((float)region.z / frameAspect);
		if (viewHeight > region.w)
		{
			viewHeight= region.w;
			viewWidth= (int)((float)region.w * frameAspect);
		}
		outViewport= glm::ivec4(region.x + (region.z - viewWidth) / 2, region.y + (region.w - viewHeight) / 2,
								viewWidth, viewHeight);
		glm::mat4 viewMatrix, projection;
		makeCameraMatchedView(m_cameraMatch.camera, m_cameraMatch.cameraToWorld, 0.01f, 100.f, viewMatrix, projection);
		outViewProjection= projection * viewMatrix;
	}
	else
	{
		outViewport= region;
		view.camera->setPerspectiveProjection(50.f, (float)region.z / (float)region.w, 0.01f, 100.f);
		outViewProjection= view.camera->getViewProjection();
	}
}

bool Scene3dPanel::drawViewImage(SceneViewState& view, bool bCameraView, bool bSideBySide, const SceneLayers& layers,
								 const TrackingFrameResult& fusedResult, const std::vector<SceneCameraView>& cameras,
								 const std::vector<const TrackingFrameResult*>& perCameraResults,
								 const SceneGizmos& gizmos, glm::vec2& outImageMin, glm::vec2& outImageSize)
{
	const ImVec2 panelSize= ImGui::GetContentRegionAvail();
	const uint16_t fbWidth= (uint16_t)std::max(64.f, panelSize.x);
	const uint16_t fbHeight= (uint16_t)std::max(64.f, panelSize.y);

	// The orbit view fills its region; the camera view letterboxes the
	// frame's aspect inside it. Side by side, the source half is the left
	// one and the avatar half, which keeps the view's own viewport for the
	// gizmos, the right, both framed from the one camera.
	std::vector<ScenePass> passes;
	if (bSideBySide)
	{
		const int leftWidth= fbWidth / 2;
		ScenePass sourcePass, avatarPass;
		splitSideBySideLayers(layers, sourcePass.layers, avatarPass.layers);
		frameRegion(view, bCameraView, glm::ivec4(0, 0, leftWidth, fbHeight), view.sourceViewport,
					view.sourceViewProjection);
		frameRegion(view, bCameraView, glm::ivec4(leftWidth, 0, fbWidth - leftWidth, fbHeight), view.viewport,
					view.viewProjection);
		sourcePass.viewport= view.sourceViewport;
		sourcePass.viewProjection= view.sourceViewProjection;
		avatarPass.viewport= view.viewport;
		avatarPass.viewProjection= view.viewProjection;
		passes.push_back(sourcePass);
		passes.push_back(avatarPass);
	}
	else
	{
		frameRegion(view, bCameraView, glm::ivec4(0, 0, fbWidth, fbHeight), view.viewport, view.viewProjection);
		view.sourceViewport= view.viewport;
		view.sourceViewProjection= view.viewProjection;
		ScenePass pass;
		pass.viewport= view.viewport;
		pass.viewProjection= view.viewProjection;
		pass.layers= layers;
		passes.push_back(pass);
	}
	if (!renderToTarget(*view.frameBuffer, fbWidth, fbHeight, passes, fusedResult, cameras, perCameraResults, gizmos))
	{
		ImGui::TextDisabled("%s", locText("scene3dPanel.rendererInitFailedText"));
		return false;
	}

	// FBO textures are bottom-up; flip V
	ImGui::Image(
		(ImTextureID)(intptr_t)view.frameBuffer->getColorTextureId(),
		ImVec2((float)fbWidth, (float)fbHeight),
		ImVec2(0, 1), ImVec2(1, 0));
	const ImVec2 imageMin= ImGui::GetItemRectMin();
	outImageMin= glm::vec2(imageMin.x, imageMin.y);
	outImageSize= glm::vec2((float)fbWidth, (float)fbHeight);

	// Each half's name at its top left, drawn over the image rather than as
	// items, so the image stays the last item for the orbit and the picks
	if (bSideBySide)
	{
		ImDrawList* drawList= ImGui::GetWindowDrawList();
		const ImU32 textColor= ImGui::GetColorU32(ImGuiCol_Text);
		const float inset= ImGui::GetStyle().ItemSpacing.x;
		drawList->AddText(ImVec2(imageMin.x + inset, imageMin.y + inset), textColor,
						  locText("scene3dPanel.sourcePoseHalfLabel"));
		drawList->AddText(ImVec2(imageMin.x + (float)(fbWidth / 2) + inset, imageMin.y + inset), textColor,
						  locText("scene3dPanel.avatarHalfLabel"));
	}
	return true;
}

void Scene3dPanel::updateOrbit(OrbitCamera& camera, bool bLeftDragOrbits)
{
	if (!ImGui::IsItemHovered())
		return;
	ImGuiIO& io= ImGui::GetIO();

	if (bLeftDragOrbits && ImGui::IsMouseDragging(ImGuiMouseButton_Left))
		camera.adjustOrbitAngles(io.MouseDelta.x * 0.4f, -io.MouseDelta.y * 0.4f);

	if (ImGui::IsMouseDragging(ImGuiMouseButton_Middle))
	{
		// Pan in the display-space horizontal plane
		const float panScale= camera.getOrbitRadius() * 0.002f;
		camera.adjustOrbitTargetPosition(glm::vec3(-io.MouseDelta.x * panScale, io.MouseDelta.y * panScale, 0.f));
	}

	if (io.MouseWheel != 0.f)
		camera.adjustOrbitRadius(-io.MouseWheel * 0.15f);
}

void Scene3dPanel::drawSkeleton(const TrackingFrameResult& result, float brightness, const glm::vec3* colorOverride)
{
	// Forward-kinematics render from the parametric pose: palm frame axes +
	// finger chains rebuilt from skeleton geometry and bend angles. This is
	// the same forward kinematics the VMC finger bones are built from, so
	// what you see here is what the stream carries.
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const HandPose& pose= result.poses[sideIndex];
		if (!pose.tracked || !pose.hasWorldPose)
			continue;

		const glm::vec3 baseColor=
			colorOverride != nullptr ? *colorOverride
									 : (pose.side == eHandSide::Left ? Colors::CornflowerBlue : Colors::Red);
		const glm::vec3 color= baseColor * brightness;

		// Palm transform in world space
		glm::mat4 palmTransform= glm::mat4_cast(pose.palmOrientationWorld);
		palmTransform[3]= glm::vec4(pose.palmPositionWorld, 1.f);

		// Palm frame axes (small: X toward fingers, Z out of the palm)
		drawTransformedAxes(*m_lineRenderer, k_displayFromWorld * palmTransform, 0.03f);

		// Palm outline: wrist -> each finger base
		const glm::vec3& middleBase= pose.skeleton.baseInPalm[(int)eFinger::Middle];
		const glm::vec3 wristInPalm(-middleBase.x, 0.f, 0.f);
		const glm::vec3 wristWorld= glm::vec3(palmTransform * glm::vec4(wristInPalm, 1.f));

		std::array<std::array<glm::vec3, 4>, FINGER_COUNT> joints;
		HandPoseModel::buildFingerJoints(palmTransform, pose.skeleton, pose.fingers, joints);

		for (int finger= 0; finger < FINGER_COUNT; ++finger)
		{
			drawSegment(*m_lineRenderer, k_displayFromWorld, wristWorld, joints[finger][0], color);
			for (int joint= 0; joint < 3; ++joint)
				drawSegment(*m_lineRenderer, k_displayFromWorld, joints[finger][joint], joints[finger][joint + 1], color);
			for (int joint= 0; joint < 4; ++joint)
				drawPoint(*m_lineRenderer, k_displayFromWorld, joints[finger][joint], Colors::White * brightness, 4.f);
		}

		// Forearm from the wrist IMU: wrist -> elbow. The DIRECTION is
		// measured, so this is a rigid extrapolation rather than the guess
		// the old geometric elbow estimate was; only the length is assumed.
		if (pose.hasForearmPose)
		{
			const glm::vec3 elbowWorld= pose.getElbowPositionWorld(m_forearmLengthMeters);
			drawSegment(*m_lineRenderer, k_displayFromWorld, wristWorld, elbowWorld, color);
			drawPoint(*m_lineRenderer, k_displayFromWorld, elbowWorld, Colors::Yellow * brightness, 7.f);

			// Forearm frame axes at the elbow, so a twisted forearm
			// (pronation/supination, which the IMU does observe) is visible
			glm::mat4 forearmTransform= glm::mat4_cast(pose.forearmOrientationWorld);
			forearmTransform[3]= glm::vec4(elbowWorld, 1.f);
			drawTransformedAxes(*m_lineRenderer, k_displayFromWorld * forearmTransform, 0.04f);

			// Vision body-pose shoulder: elbow -> shoulder completes the arm
			if (pose.hasShoulder)
			{
				drawSegment(*m_lineRenderer, k_displayFromWorld, elbowWorld, pose.shoulderPositionWorld, color);
				drawPoint(*m_lineRenderer, k_displayFromWorld, pose.shoulderPositionWorld,
						  Colors::Yellow * brightness, 7.f);
			}
		}
		else if (pose.hasShoulder)
		{
			drawPoint(*m_lineRenderer, k_displayFromWorld, pose.shoulderPositionWorld,
					  Colors::Yellow * brightness, 7.f);
		}
	}

	// Head pose from the vision body-pose solver: +X facing, +Z up
	if (result.head.valid)
	{
		glm::mat4 headTransform= glm::mat4_cast(result.head.orientationWorld);
		headTransform[3]= glm::vec4(result.head.positionWorld, 1.f);
		drawTransformedAxes(*m_lineRenderer, k_displayFromWorld * headTransform, 0.06f);
		drawPoint(*m_lineRenderer, k_displayFromWorld, result.head.positionWorld,
				  Colors::White * brightness, 6.f);
	}
}

bool Scene3dPanel::drawAvatar(const glm::mat4& viewProjection, bool bShow)
{
	if (m_avatarModel == nullptr || m_avatarSkeleton == nullptr)
	{
		// Unloaded: drop the GPU copy rather than keeping a 12 MB model resident
		m_meshView.clear();
		return false;
	}
	if (!bShow)
		return false;

	// The pose carries the root placement, so the model matrix is only the
	// avatar's own axis convention and the display rotation every
	// world-space drawing takes
	if (m_bHasAvatarPose)
		computePosedGlobals(*m_avatarModel, *m_avatarSkeleton, m_avatarPose, m_posedGlobals);
	else
		m_posedGlobals= m_avatarSkeleton->getRestGlobalsAvatar();
	const glm::mat4 modelMatrix= k_displayFromWorld * m_avatarSkeleton->getWorldFromAvatar();

	// One key light from above and in front of the avatar (display space:
	// +Y up, +X the avatar's facing direction)
	const glm::vec3 lightDirection= glm::normalize(glm::vec3(0.6f, 1.f, 0.4f));
	// The rest face unless the frame's values came from the binding built
	// for this model
	if (m_faceBinding != nullptr && m_faceBinding->fitsModel(*m_avatarModel))
	{
		const bool bFace= m_bHasAvatarFace && m_avatarFace.outputs.generation == m_faceBinding->getGeneration();
		m_faceBinding->evaluateMorphs(bFace ? &m_avatarFace.outputs.targetValues : nullptr, m_morphWeights);
	}
	else
	{
		m_morphWeights.resize(m_avatarModel->meshes.size());
		for (size_t meshIndex= 0; meshIndex < m_avatarModel->meshes.size(); ++meshIndex)
			m_morphWeights[meshIndex].assign(m_avatarModel->meshes[meshIndex].morphTargetNames.size(), 0.f);
	}
	return m_meshView.draw(viewProjection, modelMatrix, m_posedGlobals, lightDirection, m_morphWeights);
}

void Scene3dPanel::drawDirectSolve()
{
	if (!m_bHasDirectSolve)
		return;
	// The shoulder line and each solved chain, the measured points in
	// yellow, carried into the avatar's world
	const glm::mat4 transform= k_displayFromWorld * m_solveToWorld;
	drawSegment(*m_lineRenderer, transform, m_directSolve.shoulder[0], m_directSolve.shoulder[1], Colors::Yellow);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const DirectRetarget::ArmChain& chain= m_directSolve.arms[sideIndex].chain;
		if (!chain.valid)
			continue;
		const glm::vec3 color= sideIndex == 0 ? Colors::CornflowerBlue : Colors::Red;
		drawSegment(*m_lineRenderer, transform, chain.shoulder, chain.elbow, color);
		drawSegment(*m_lineRenderer, transform, chain.elbow, chain.wrist, color);
		drawPoint(*m_lineRenderer, transform, chain.elbow, Colors::Yellow, 7.f);
		drawPoint(*m_lineRenderer, transform, chain.wrist, Colors::Yellow, 7.f);
	}
}

bool Scene3dPanel::projectToImage(const glm::vec3& world, const glm::vec2& imageMin, const glm::vec2& imageSize,
								  glm::vec2& outPixel) const
{
	return projectToViewImage(m_mainView.viewProjection, m_mainView.viewport, imageMin, imageSize.y, world, outPixel);
}

bool Scene3dPanel::captureFromCamera(const TrackingFrameResult& fusedResult,
									 const std::vector<SceneCameraView>& cameras,
									 const std::vector<const TrackingFrameResult*>& perCameraResults, cv::Mat& outBgr)
{
	const DirectCameraModel& camera= m_cameraMatch.camera;
	if (!m_cameraMatch.valid || camera.width <= 0 || camera.height <= 0 || camera.width > 0xFFFF ||
		camera.height > 0xFFFF || !startRenderers())
		return false;
	const uint16_t width= (uint16_t)camera.width;
	const uint16_t height= (uint16_t)camera.height;

	glm::mat4 view, projection;
	makeCameraMatchedView(camera, m_cameraMatch.cameraToWorld, 0.01f, 100.f, view, projection);

	// What the scene drew before it had layers: everything but the gizmos,
	// the avatar and the per-camera skeletons as their settings say
	SceneLayers layers;
	layers.bAvatar= m_bShowAvatar;
	layers.bPerCameraSkeletons= m_bShowPerCameraSkeletons;
	layers.bGizmos= false;
	const SceneGizmos noGizmos;
	if (!renderToTarget(*m_captureBuffer, width, height, glm::ivec4(0, 0, width, height), projection * view, layers,
						fusedResult, cameras, perCameraResults, noGizmos))
		return false;

	// Read back and flip: GL rows run bottom-up
	m_captureBuffer->bindFrameBuffer();
	cv::Mat rgba(height, width, CV_8UC4);
	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(0, 0, width, height, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data);
	m_captureBuffer->unbindFrameBuffer();
	cv::cvtColor(rgba, outBgr, cv::COLOR_RGBA2BGR);
	cv::flip(outBgr, outBgr, 0);
	return true;
}

bool Scene3dPanel::updateGizmoDrag(SceneGizmos& gizmos, const glm::vec2& imageMin, const glm::vec2& imageSize)
{
	constexpr float kPickRadiusPixels= 8.f;
	const ImVec2 mouse= ImGui::GetIO().MousePos;
	const glm::vec2 mousePixel(mouse.x, mouse.y);

	if (m_draggedElbowHint >= 0 &&
		(!ImGui::IsMouseDown(ImGuiMouseButton_Left) || !gizmos.elbowHints[m_draggedElbowHint].bEnabled))
	{
		m_draggedElbowHint= -1;
	}

	// A press on a hint starts its drag, in the camera-facing plane through it
	if (m_draggedElbowHint < 0 && ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
	{
		float bestDistance= kPickRadiusPixels;
		for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
		{
			const SceneGizmos::ElbowHint& hint= gizmos.elbowHints[sideIndex];
			glm::vec2 pixel;
			if (!hint.bEnabled || !projectToImage(hint.hintWorld, imageMin, imageSize, pixel))
				continue;
			const float distance= glm::length(pixel - mousePixel);
			if (distance <= bestDistance)
			{
				bestDistance= distance;
				m_draggedElbowHint= sideIndex;
			}
		}
		if (m_draggedElbowHint >= 0)
		{
			m_dragPlanePoint= glm::vec3(k_displayFromWorld * glm::vec4(gizmos.elbowHints[m_draggedElbowHint].hintWorld, 1.f));
			m_dragPlaneNormal= glm::normalize(m_mainView.camera->getCameraPosition() - m_dragPlanePoint);
		}
	}
	if (m_draggedElbowHint < 0)
		return false;

	// Move the hint to where the mouse ray meets the drag plane
	SceneGizmos::ElbowHint& hint= gizmos.elbowHints[m_draggedElbowHint];
	hint.bDragging= true;
	hint.draggedHintWorld= hint.hintWorld;
	// The ray through the mouse within the view's viewport, which side by
	// side is the avatar half
	const glm::ivec4& viewport= m_mainView.viewport;
	const float viewportLeft= imageMin.x + (float)viewport.x;
	const float viewportTop= imageMin.y + imageSize.y - (float)(viewport.y + viewport.w);
	const float ndcX= (mousePixel.x - viewportLeft) / (float)viewport.z * 2.f - 1.f;
	const float ndcY= 1.f - (mousePixel.y - viewportTop) / (float)viewport.w * 2.f;
	glm::vec3 rayOrigin, rayDirection;
	m_mainView.camera->unprojectRay(ndcX, ndcY, rayOrigin, rayDirection);
	const float denominator= glm::dot(rayDirection, m_dragPlaneNormal);
	if (std::fabs(denominator) > 1e-6f)
	{
		const float distance= glm::dot(m_dragPlanePoint - rayOrigin, m_dragPlaneNormal) / denominator;
		if (distance > 0.f)
		{
			const glm::vec3 displayPoint= rayOrigin + rayDirection * distance;
			hint.draggedHintWorld= glm::vec3(glm::inverse(k_displayFromWorld) * glm::vec4(displayPoint, 1.f));
		}
	}
	return true;
}

void Scene3dPanel::drawGizmos(const SceneGizmos& gizmos)
{
	// Rings face the camera: the circle helper draws in its local XZ plane,
	// so local Y goes along the view direction (the orbit camera's: the
	// gizmos are not drawn in the camera view)
	const glm::mat3 cameraRotation= glm::transpose(glm::mat3(m_mainView.camera->getViewMatrix()));
	const glm::mat3 facing(cameraRotation[0], cameraRotation[2], cameraRotation[1]);
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		const SceneGizmos::ElbowHint& hint= gizmos.elbowHints[sideIndex];
		if (!hint.bEnabled)
			continue;
		const glm::vec3 color= sideIndex == 0 ? Colors::CornflowerBlue : Colors::Red;
		const glm::vec3 ringColor= sideIndex == m_draggedElbowHint ? Colors::White : Colors::Yellow;
		drawSegment(*m_lineRenderer, k_displayFromWorld, hint.shoulderWorld, hint.hintWorld, color);
		glm::mat4 ring(facing);
		ring[3]= k_displayFromWorld * glm::vec4(hint.hintWorld, 1.f);
		drawTransformedCircle(*m_lineRenderer, ring, 0.025f, ringColor, 24);
		drawPoint(*m_lineRenderer, k_displayFromWorld, hint.hintWorld, ringColor, 6.f);
	}
}

void Scene3dPanel::updateSourcePick(const glm::vec2& imageMin, const glm::vec2& imageSize)
{
	// Press and release positions alone, never the per-frame mouse delta, so
	// an injected click picks as a real one does
	const ImVec2 mouse= ImGui::GetIO().MousePos;
	const glm::vec2 mousePixel(mouse.x, mouse.y);
	if (ImGui::IsItemHovered() && ImGui::IsMouseClicked(ImGuiMouseButton_Left))
	{
		m_bPickPressArmed= m_draggedElbowHint < 0;
		m_pickPressPixel= mousePixel;
	}
	if (!m_bPickPressArmed || !ImGui::IsMouseReleased(ImGuiMouseButton_Left))
		return;
	m_bPickPressArmed= false;
	if (glm::length(mousePixel - m_pickPressPixel) > kClickSlopPixels)
		return;
	eRetargetChain chain;
	if (pickSourceSkeleton(getSourceSkeleton(), m_mainView.sourceViewProjection, m_mainView.sourceViewport, imageMin,
						   imageSize.y, mousePixel, kSourcePickTolerancePixels, chain))
	{
		m_bHasPickedChain= true;
		m_pickedChain= chain;
	}
}

void Scene3dPanel::drawSourceSkeleton()
{
	const SourceSkeleton& skeleton= getSourceSkeleton();
	std::vector<std::array<glm::vec3, 2>> dashes;
	for (const SourceSkeleton::Segment& segment : skeleton.segments)
	{
		const bool bSelected= segment.chain == m_selectedChain;
		// The selected chain brighter, inside a ring of halo lines
		const glm::vec3 color= bSelected ? segment.color + (Colors::White - segment.color) * 0.4f : segment.color;
		dashes.clear();
		if (segment.bDashed)
			appendDashes(segment.start, segment.end, SourceSkeletonStyle::kDashLength, SourceSkeletonStyle::kGapLength,
						 dashes);
		else
			dashes.push_back({segment.start, segment.end});

		glm::vec3 haloA(0.f), haloB(0.f);
		if (bSelected)
		{
			const glm::vec3 delta= segment.end - segment.start;
			const float length= glm::length(delta);
			if (length > 1e-6f)
			{
				const glm::vec3 direction= delta / length;
				const glm::vec3 reference=
					std::fabs(direction.z) < 0.9f ? glm::vec3(0.f, 0.f, 1.f) : glm::vec3(1.f, 0.f, 0.f);
				haloA= glm::normalize(glm::cross(direction, reference)) * kHaloRadius;
				haloB= glm::cross(direction, haloA);
			}
		}
		for (const std::array<glm::vec3, 2>& dash : dashes)
		{
			drawSegment(*m_lineRenderer, k_displayFromWorld, dash[0], dash[1], color);
			if (bSelected)
			{
				for (const glm::vec3& offset : {haloA, -haloA, haloB, -haloB})
					drawSegment(*m_lineRenderer, k_displayFromWorld, dash[0] + offset, dash[1] + offset, k_haloColor);
			}
		}
	}
	for (const SourceSkeleton::Marker& marker : skeleton.markers)
	{
		const bool bSelected= marker.chain == m_selectedChain;
		const glm::vec3 color= bSelected ? marker.color + (Colors::White - marker.color) * 0.4f : marker.color;
		drawPoint(*m_lineRenderer, k_displayFromWorld, marker.position, color, bSelected ? marker.size + 3.f : marker.size);
	}
}

void Scene3dPanel::drawIkGoals()
{
	// A cross along the world axes and a point where the solve aimed each
	// arm's wrist
	if (!m_bHasSourcePose)
		return;
	for (int sideIndex= 0; sideIndex < 2; ++sideIndex)
	{
		if (!m_sourceGoals.bHasWristGoal[sideIndex])
			continue;
		const glm::vec3& goal= m_sourceGoals.wristGoalWorld[sideIndex];
		for (int axis= 0; axis < 3; ++axis)
		{
			glm::vec3 offset(0.f);
			offset[axis]= kIkGoalHalfSize;
			drawSegment(*m_lineRenderer, k_displayFromWorld, goal - offset, goal + offset, k_ikGoalColor);
		}
		drawPoint(*m_lineRenderer, k_displayFromWorld, goal, k_ikGoalColor, 6.f);
	}
}

void Scene3dPanel::renderScene(const TrackingFrameResult& fusedResult, const std::vector<SceneCameraView>& cameras,
							   const std::vector<const TrackingFrameResult*>& perCameraResults,
							   const SceneGizmos& gizmos, const glm::mat4& viewProjection,
							   const SceneLayers& layers)
{
	glEnable(GL_DEPTH_TEST);

	if (layers.bGrid)
	{
		// Marker plane grid (world XY plane -> display XZ plane, which is what drawGrid expects)
		drawGrid(*m_lineRenderer, glm::mat4(1.f), 2.f, 2.f, 20, 20, Colors::DarkGray);

		// Marker axes at the world origin
		drawTransformedAxes(*m_lineRenderer, k_displayFromWorld, 0.1f);
	}

	// One frustum per calibrated camera (cameraToWorld maps CV-convention
	// camera space to world; the frustum helper draws in GL convention)
	for (size_t cameraIndex= 0; layers.bCameraFrustums && cameraIndex < cameras.size(); ++cameraIndex)
	{
		const SceneCameraView& view= cameras[cameraIndex];
		if (!view.bHasExtrinsics || view.intrinsics == nullptr)
			continue;

		const glm::vec3& color= k_cameraColors[cameraIndex % 4];
		const glm::mat4 cameraXform= k_displayFromWorld * view.cameraToWorld * k_glFromCvFlip;
		drawTransformedFrustum(
			*m_lineRenderer, cameraXform,
			(float)view.intrinsics->hfov, (float)view.intrinsics->vfov,
			0.05f, 0.5f,
			color);
		drawTransformedAxes(*m_lineRenderer, cameraXform, 0.05f);
	}

	// Dimmed per-camera skeletons (world-space agreement check across cameras)
	if (layers.bPerCameraSkeletons)
	{
		for (size_t cameraIndex= 0; cameraIndex < perCameraResults.size(); ++cameraIndex)
		{
			if (perCameraResults[cameraIndex] != nullptr)
			{
				const glm::vec3 color= k_cameraColors[cameraIndex % 4];
				drawSkeleton(*perCameraResults[cameraIndex], 0.45f, &color);
			}
		}
	}

	// Fused skeleton, full brightness
	if (layers.bFusedSkeleton)
		drawSkeleton(fusedResult, 1.f, nullptr);
	if (layers.bSourcePose)
		drawSourceSkeleton();
	if (layers.bIkGoals)
		drawIkGoals();
	if (layers.bDirectSolve)
		drawDirectSolve();
	if (layers.bGizmos)
		drawGizmos(gizmos);

	// The avatar mesh first, then the lines over it with depth off when it is
	// shown, so the tracked skeleton stays visible inside the character
	const bool bAvatarDrawn= drawAvatar(viewProjection, layers.bAvatar);
	m_lineRenderer->render3d(viewProjection, bAvatarDrawn);
}

std::vector<SceneCameraView> makeSceneCameraViews(const AppConfig& config)
{
	std::vector<SceneCameraView> cameras;
	for (size_t cameraIndex= 0; cameraIndex < config.cameraCount(); ++cameraIndex)
	{
		const CameraProfile& profile= config.camera(cameraIndex);
		SceneCameraView view;
		view.cameraToWorld= glm::mat4(profile.extrinsics.markerFromCamera);
		view.bHasExtrinsics= profile.extrinsics.present;
		view.intrinsics= profile.intrinsics.present ? &profile.intrinsics.intrinsics : nullptr;
		cameras.push_back(view);
	}
	return cameras;
}
