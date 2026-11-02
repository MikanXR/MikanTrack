#include "HeadlessGlContext.h"

#include "GL/glew.h"
#include "SDL.h"

#include "Logger.h"

HeadlessGlContext::~HeadlessGlContext()
{
	destroy();
}

bool HeadlessGlContext::create(const char* windowTitle)
{
	destroy();
	if (SDL_Init(SDL_INIT_VIDEO) != 0)
	{
		MIKAN_LOG_ERROR("HeadlessGlContext") << "SDL_Init failed: " << SDL_GetError();
		return false;
	}
	m_bSdlInitialized= true;
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MAJOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_MINOR_VERSION, 3);
	SDL_GL_SetAttribute(SDL_GL_CONTEXT_PROFILE_MASK, SDL_GL_CONTEXT_PROFILE_CORE);
	SDL_GL_SetAttribute(SDL_GL_DEPTH_SIZE, 24);
	m_window= SDL_CreateWindow(windowTitle, SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, 64, 64,
							   SDL_WINDOW_OPENGL | SDL_WINDOW_HIDDEN);
	m_context= m_window != nullptr ? SDL_GL_CreateContext(m_window) : nullptr;
	if (m_context == nullptr)
	{
		MIKAN_LOG_ERROR("HeadlessGlContext") << "GL context failed: " << SDL_GetError();
		destroy();
		return false;
	}
	SDL_GL_MakeCurrent(m_window, m_context);
	glewExperimental= GL_TRUE;
	if (glewInit() != GLEW_OK)
	{
		MIKAN_LOG_ERROR("HeadlessGlContext") << "glewInit failed";
		destroy();
		return false;
	}
	MIKAN_LOG_INFO("HeadlessGlContext") << "OpenGL " << (const char*)glGetString(GL_VERSION) << " on "
										<< (const char*)glGetString(GL_RENDERER);
	return true;
}

void HeadlessGlContext::destroy()
{
	if (m_context != nullptr)
	{
		SDL_GL_DeleteContext(m_context);
		m_context= nullptr;
	}
	if (m_window != nullptr)
	{
		SDL_DestroyWindow(m_window);
		m_window= nullptr;
	}
	if (m_bSdlInitialized)
	{
		SDL_Quit();
		m_bSdlInitialized= false;
	}
}
