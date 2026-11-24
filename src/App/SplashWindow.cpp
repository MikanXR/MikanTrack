#include "SplashWindow.h"

#include <cstdint>

#include "SDL.h"

#include "opencv2/core.hpp"
#include "opencv2/imgcodecs.hpp"

#include "Logger.h"

#ifdef _WIN32
#include <windows.h>
#endif

// The name the resource script gives the splash image
static const char* k_splashResourceName= "MIKAN_SPLASH";

// The splash image as decoded BGR pixels, empty when the executable carries
// none or it does not decode
static cv::Mat loadSplashImage()
{
#ifdef _WIN32
	const HMODULE module= GetModuleHandle(nullptr);
	const HRSRC resourceInfo= FindResourceA(module, k_splashResourceName, MAKEINTRESOURCEA(10)); // RT_RCDATA
	const HGLOBAL resourceHandle= resourceInfo != nullptr ? LoadResource(module, resourceInfo) : nullptr;
	const void* encodedBytes= resourceHandle != nullptr ? LockResource(resourceHandle) : nullptr;
	if (encodedBytes == nullptr)
	{
		MIKAN_LOG_WARNING("SplashWindow::open") << "No splash image in the executable";
		return cv::Mat();
	}

	// The header only borrows the resource bytes, which live as long as the module
	const cv::Mat encoded(1, (int)SizeofResource(module, resourceInfo), CV_8UC1, const_cast<void*>(encodedBytes));
	cv::Mat image= cv::imdecode(encoded, cv::IMREAD_COLOR);
	if (image.empty())
	{
		MIKAN_LOG_WARNING("SplashWindow::open") << "Unable to decode the splash image";
	}
	return image;
#else
	return cv::Mat();
#endif
}

// SDL reports the display's effective DPI, which is the Windows scale factor
// against 96. A process that is not DPI aware reads 96 and Windows scales the
// window itself.
static float getPrimaryDisplayContentScale()
{
	float ddpi= 0.f;
	if (SDL_GetDisplayDPI(0, &ddpi, nullptr, nullptr) != 0 || ddpi < 96.f)
		return 1.f;

	return ddpi / 96.f;
}

SplashWindow::~SplashWindow()
{
	close();
}

bool SplashWindow::open(const char* title)
{
	close();

	const cv::Mat image= loadSplashImage();
	if (image.empty())
		return false;

	const float displayScale= getPrimaryDisplayContentScale();
	m_sdlWindow= SDL_CreateWindow(
		title,
		SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
		(int)(image.cols * displayScale), (int)(image.rows * displayScale),
		SDL_WINDOW_BORDERLESS | SDL_WINDOW_ALLOW_HIGHDPI);
	if (m_sdlWindow == nullptr)
	{
		MIKAN_LOG_WARNING("SplashWindow::open") << "Unable to create the splash window: " << SDL_GetError();
		return false;
	}

	// The surface only borrows the decoded pixels, and the blit below is the copy
	SDL_Surface* imageSurface= SDL_CreateRGBSurfaceWithFormatFrom(
		image.data, image.cols, image.rows, 24, (int)image.step, SDL_PIXELFORMAT_BGR24);
	SDL_Surface* windowSurface= SDL_GetWindowSurface(m_sdlWindow);
	const bool bDrawn= imageSurface != nullptr && windowSurface != nullptr &&
		SDL_BlitScaled(imageSurface, nullptr, windowSurface, nullptr) == 0 &&
		SDL_UpdateWindowSurface(m_sdlWindow) == 0;
	if (imageSurface != nullptr)
		SDL_FreeSurface(imageSurface);

	if (!bDrawn)
	{
		MIKAN_LOG_WARNING("SplashWindow::open") << "Unable to draw the splash image: " << SDL_GetError();
		close();
		return false;
	}

	// Lets the window manager show the window before startup carries on
	SDL_PumpEvents();
	m_openTime= std::chrono::steady_clock::now();

	return true;
}

void SplashWindow::close()
{
	if (m_sdlWindow != nullptr)
	{
		SDL_DestroyWindow(m_sdlWindow);
		m_sdlWindow= nullptr;
	}
}

float SplashWindow::getSecondsOpen() const
{
	if (m_sdlWindow == nullptr)
		return 0.f;

	return std::chrono::duration<float>(std::chrono::steady_clock::now() - m_openTime).count();
}
