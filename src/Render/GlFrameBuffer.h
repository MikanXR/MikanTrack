#pragma once

#include <cstdint>

// Offscreen render target for the 3D scene viewport:
// RGBA8 color texture + 24-bit depth renderbuffer.
// The color texture id is handed to ImGui::Image for display.
class GlFrameBuffer
{
public:
	GlFrameBuffer()= default;
	~GlFrameBuffer();

	GlFrameBuffer(const GlFrameBuffer&)= delete;
	GlFrameBuffer& operator=(const GlFrameBuffer&)= delete;

	// Creates the FBO + attachments. Requires a current GL context.
	bool init(uint16_t width, uint16_t height);

	// Recreates the FBO at a new resolution (no-op if the size is unchanged)
	bool resize(uint16_t width, uint16_t height);

	// Destroys the FBO and its attachments
	void dispose();

	// Binds the FBO for rendering and sets the viewport to cover it. The
	// binding it replaces is tracked rather than queried from GL, since a
	// query is a round trip to the driver.
	void bindFrameBuffer();

	// Rebinds the GlFrameBuffer this one replaced with its full viewport, or
	// the default framebuffer, whose viewport is left to whoever draws there
	// next
	void unbindFrameBuffer();

	uint32_t getColorTextureId() const { return m_glColorTextureId; }
	uint16_t getWidth() const { return m_width; }
	uint16_t getHeight() const { return m_height; }
	bool getIsValid() const { return m_glFrameBufferId != 0; }

private:
	uint32_t m_glFrameBufferId= 0;
	uint32_t m_glColorTextureId= 0;
	uint32_t m_glDepthRenderBufferId= 0;
	uint16_t m_width= 0;
	uint16_t m_height= 0;

	// Binds this FBO, or the default framebuffer for null, with the full
	// viewport of an FBO
	static void bindTarget(const GlFrameBuffer* target);

	// The GlFrameBuffer bound when this one was, null for the default
	// framebuffer
	GlFrameBuffer* m_prevBound= nullptr;
	bool m_bIsBound= false;

	// The GlFrameBuffer currently bound on the GL thread, null for the default
	// framebuffer
	static GlFrameBuffer* s_currentBound;
};
