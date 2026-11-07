#include "TestCommon.h"

#include "GL/glew.h"

#include "AvatarFaceMap.h"
#include "AvatarRig.h"
#include "AvatarSkeleton.h"
#include "AudioTypes.h"
#include "GlFrameBuffer.h"
#include "GlSkinnedMeshRenderer.h"
#include "HeadlessGlContext.h"
#include "MathGLM.h"
#include "OrbitCamera.h"
#include "PathUtils.h"
#include "Scene3dPanel.h"
#include "VisemeTable.h"
#include "VrmLoader.h"

// Renders a contact sheet of an avatar's face under the viseme frames a
// bench JSON holds, through the same renderer and face map the 3D scene
// uses, from a hidden window: the frame's visemes onto a neutral face
// through the viseme table, and the visemes themselves to the face map, the
// way the live merge hands them on. The headless way to look at what the
// backend's weights do to a real face: tiles are spread evenly over the
// clip, each labelled with its frame index and its dominant viseme.
//
//   --render-mouth <file.vrm> <bench.json> [out.png] [tiles] [chunked|offline|filtered]
static int runRenderMouthTool(const TestArgs& args)
{
	if (args.size() < 2)
	{
		MIKAN_LOG_ERROR("render-mouth")
			<< "Usage: --render-mouth <file.vrm> <bench.json> [out.png] [tiles] [chunked|offline|filtered]";
		return 1;
	}
	const std::filesystem::path vrmPath= PathUtils::utf8ToPath(args[0]);
	const std::filesystem::path jsonPath= PathUtils::utf8ToPath(args[1]);
	const std::filesystem::path outPath= args.size() > 2 ? PathUtils::utf8ToPath(args[2]) : "render-mouth.png";
	const int tileCount= args.size() > 3 ? std::max(1, std::atoi(args[3].c_str())) : 12;
	const std::string series= args.size() > 4 ? args[4] : "chunked";

	nlohmann::json bench;
	{
		std::ifstream file(jsonPath);
		if (!file)
		{
			MIKAN_LOG_ERROR("render-mouth") << "cannot open " << jsonPath.string();
			return 1;
		}
		try
		{
			file >> bench;
		}
		catch (const std::exception& e)
		{
			MIKAN_LOG_ERROR("render-mouth") << "bad JSON: " << e.what();
			return 1;
		}
	}
	if (!bench.contains(series) || !bench[series].is_array() || bench[series].empty())
	{
		MIKAN_LOG_ERROR("render-mouth") << "no '" << series << "' frames in " << jsonPath.string();
		return 1;
	}
	const nlohmann::json& frames= bench[series];

	const VrmLoader::LoadResult loaded= VrmLoader::loadFile(vrmPath);
	if (loaded.model == nullptr)
	{
		MIKAN_LOG_ERROR("render-mouth") << "Load failed: " << loaded.error;
		return 1;
	}
	const AvatarRigSettings rig= loadAvatarRig(vrmPath);
	std::vector<std::string> rigWarnings;
	applyRigToModel(rig, *loaded.model, rigWarnings);
	const AvatarSkeleton skeleton(*loaded.model);
	const AvatarFaceMorphs faceMorphs(*loaded.model, AvatarFaceMap::build(*loaded.model));

	HeadlessGlContext gl;
	if (!gl.create("render-mouth"))
		return 1;

	int exitCode= 1;
	{
		constexpr uint16_t kTile= 256;
		GlFrameBuffer frameBuffer;
		GlSkinnedMeshRenderer meshRenderer;
		if (!frameBuffer.init(kTile, kTile) || !meshRenderer.startup() || !meshRenderer.upload(*loaded.model))
		{
			MIKAN_LOG_ERROR("render-mouth") << "Renderer setup failed (see GL errors above)";
		}
		else
		{
			// Frame the head from the front: the avatar faces world +X, and
			// the orbit camera's yaw 0 looks down display -Z, so a quarter
			// turn puts it in front of the face
			const float headHeight= skeleton.getBone(eHumanoidBone::Head).restPositionWorld.z;
			OrbitCamera camera;
			camera.setOrbitTargetPosition(glm::vec3(0.f, headHeight + 0.06f, 0.f));
			camera.setOrbitLocation(90.f, 0.f, 0.45f);
			camera.setPerspectiveProjection(30.f, 1.f, 0.01f, 100.f);
			const glm::mat4 modelMatrix= displayFromWorld() * skeleton.getWorldFromAvatar();
			const std::vector<glm::mat4> globals= skeleton.getRestGlobalsAvatar();
			const glm::vec3 lightDirection= glm::normalize(glm::vec3(0.3f, 1.f, 0.8f));

			const int columns= (int)std::ceil(std::sqrt((double)tileCount));
			const int rows= (tileCount + columns - 1) / columns;
			cv::Mat sheet(rows * kTile, columns * kTile, CV_8UC3, cv::Scalar(20, 20, 24));
			std::vector<std::vector<float>> morphWeights;
			cv::Mat rgba(kTile, kTile, CV_8UC4);

			for (int tile= 0; tile < tileCount; ++tile)
			{
				const size_t frameIndex= tileCount > 1 ? (frames.size() - 1) * (size_t)tile / (size_t)(tileCount - 1) : 0;
				const nlohmann::json& row= frames[frameIndex];
				VisemeWeights visemes{};
				for (int viseme= 0; viseme < VISEME_COUNT && viseme < (int)row.size(); ++viseme)
					visemes[viseme]= row[(size_t)viseme].get<float>();
				std::array<float, ARKIT_BLENDSHAPE_COUNT> face{};
				applyVisemeWeights(visemes, face);
				faceMorphs.evaluate(&face, &visemes, morphWeights);
				meshRenderer.setMorphWeights(morphWeights);

				frameBuffer.bindFrameBuffer();
				glClearColor(0.08f, 0.08f, 0.1f, 1.f);
				glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
				glEnable(GL_DEPTH_TEST);
				meshRenderer.draw(camera.getViewProjection(), modelMatrix, globals, lightDirection);
				glPixelStorei(GL_PACK_ALIGNMENT, 1);
				glReadPixels(0, 0, kTile, kTile, GL_RGBA, GL_UNSIGNED_BYTE, rgba.data);
				frameBuffer.unbindFrameBuffer();

				cv::Mat bgr;
				cv::cvtColor(rgba, bgr, cv::COLOR_RGBA2BGR);
				cv::flip(bgr, bgr, 0);
				const int dominant= dominantViseme(visemes);
				const std::string label= "#" + std::to_string(frameIndex) + " " + visemeName(dominant) + " " +
										 std::to_string(visemes[dominant]).substr(0, 4);
				cv::putText(bgr, label, cv::Point(6, 20), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(255, 255, 255), 1);
				bgr.copyTo(sheet(cv::Rect((tile % columns) * kTile, (tile / columns) * kTile, kTile, kTile)));
			}

			if (cv::imwrite(outPath.string(), sheet))
			{
				MIKAN_LOG_INFO("render-mouth") << "Wrote " << outPath << " (" << tileCount << " of " << frames.size()
											   << " " << series << " frames)";
				exitCode= 0;
			}
			else
			{
				MIKAN_LOG_ERROR("render-mouth") << "Could not write " << outPath;
			}
		}
		meshRenderer.shutdown();
		frameBuffer.dispose();
	}
	return exitCode;
}
MIKAN_REGISTER_TEST("--render-mouth",
					"Render a contact sheet of an avatar's face under a bench JSON's viseme frames (hidden window)",
					eTestCategory::Tool, runRenderMouthTool);
