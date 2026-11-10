#include "AvatarMeshView.h"

#include "AvatarTypes.h"
#include "GlSkinnedMeshRenderer.h"

AvatarMeshView::AvatarMeshView()
	: m_renderer(std::make_unique<GlSkinnedMeshRenderer>())
{}

AvatarMeshView::~AvatarMeshView()= default;

void AvatarMeshView::setAvatar(std::shared_ptr<const AvatarModel> model, uint32_t generation)
{
	m_model= std::move(model);
	m_generation= generation;
}

bool AvatarMeshView::draw(const glm::mat4& viewProjection, const glm::mat4& modelMatrix,
						  const std::vector<glm::mat4>& nodeGlobalsAvatar, const glm::vec3& lightDirection,
						  const std::vector<std::vector<float>>& morphWeights)
{
	if (m_model == nullptr)
	{
		// Unloaded: drop the GPU copy rather than keeping a 12 MB model resident
		clear();
		return false;
	}

	if (!m_bRendererInitialized)
	{
		m_bRendererInitialized= m_renderer->startup();
		if (!m_bRendererInitialized)
			return false;
	}
	// Rebuild the GPU copy only when the model changed, not per frame
	if (m_uploadedGeneration != m_generation || !m_renderer->hasModel())
	{
		if (!m_renderer->upload(*m_model))
			return false;
		m_uploadedGeneration= m_generation;
	}

	m_renderer->setMorphWeights(morphWeights);
	m_renderer->draw(viewProjection, modelMatrix, nodeGlobalsAvatar, lightDirection);
	return true;
}

void AvatarMeshView::clear()
{
	if (m_renderer->hasModel())
		m_renderer->clear();
}
