#pragma once

struct SDL_Window;

// A GL context on a window that never shows: the same context the app
// creates, for the tools that render through the scene renderer without
// the UI. One per process; create() logs what failed.
class HeadlessGlContext
{
public:
	HeadlessGlContext()= default;
	~HeadlessGlContext();

	HeadlessGlContext(const HeadlessGlContext&)= delete;
	HeadlessGlContext& operator=(const HeadlessGlContext&)= delete;

	// Initializes SDL video, opens the hidden window, creates and makes
	// current a core 3.3 context, loads GLEW
	bool create(const char* windowTitle);
	void destroy();
	bool isValid() const { return m_context != nullptr; }

private:
	SDL_Window* m_window= nullptr;
	void* m_context= nullptr;
	bool m_bSdlInitialized= false;
};
