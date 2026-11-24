#pragma once

#include <chrono>

struct SDL_Window;

// The image shown while the app starts: a borderless window whose surface
// holds the splash image compiled into the executable. It has no GL context
// and no render loop, so it can open before anything else is set up.
class SplashWindow
{
public:
	SplashWindow()= default;
	~SplashWindow();

	SplashWindow(const SplashWindow&)= delete;
	SplashWindow& operator=(const SplashWindow&)= delete;

	// Needs SDL's video subsystem. A missing or undecodable image leaves the
	// window closed, which the app runs without.
	bool open(const char* title);
	void close();

	bool isOpen() const { return m_sdlWindow != nullptr; }
	// Time since open() put the image on screen, 0 when closed
	float getSecondsOpen() const;

private:
	SDL_Window* m_sdlWindow= nullptr;
	std::chrono::steady_clock::time_point m_openTime;
};
