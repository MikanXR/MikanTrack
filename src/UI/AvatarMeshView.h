#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "glm/ext/matrix_float4x4.hpp"
#include "glm/ext/vector_float3.hpp"

struct AvatarModel;
class GlSkinnedMeshRenderer;

// One GPU copy of the avatar mesh and the renderer that draws it: lazy
// startup on first draw, re-upload when the model generation changes,
// release when the model goes away. The caller supplies the pose globals and
// the morph weights per frame.
class AvatarMeshView
{
public:
	AvatarMeshView();
	~AvatarMeshView();

	AvatarMeshView(const AvatarMeshView&)= delete;
	AvatarMeshView& operator=(const AvatarMeshView&)= delete;

	// The model to draw (null: none) and the counter that tells a changed
	// model from the one already uploaded
	void setAvatar(std::shared_ptr<const AvatarModel> model, uint32_t generation);
	bool hasModel() const { return m_model != nullptr; }

	// Draws into the bound framebuffer; false when nothing could be drawn (no
	// model, startup or upload failed). Enables depth testing itself as the
	// renderer does.
	bool draw(const glm::mat4& viewProjection, const glm::mat4& modelMatrix,
			  const std::vector<glm::mat4>& nodeGlobalsAvatar, const glm::vec3& lightDirection,
			  const std::vector<std::vector<float>>& morphWeights);
	// Releases the GPU copy
	void clear();

private:
	std::unique_ptr<GlSkinnedMeshRenderer> m_renderer;
	bool m_bRendererInitialized= false;
	std::shared_ptr<const AvatarModel> m_model;
	uint32_t m_generation= 0;
	uint32_t m_uploadedGeneration= 0;
};
