#include <string>

#include "App.h"
#include "TestRegistry.h"

#ifdef _WIN32
#include <windows.h>
#endif

// Asks a hybrid-GPU laptop's graphics driver to run this process on the
// discrete GPU. Without these, Optimus and AMD switchable graphics start the
// executable's OpenGL on the integrated GPU. The drivers only read the symbols
// from the executable's own export table, so they live in this file.
#ifdef _WIN32
extern "C"
{
	__declspec(dllexport) unsigned long NvOptimusEnablement= 0x00000001;
	__declspec(dllexport) int AmdPowerXpressRequestHighPerformance= 1;
}
#endif

// Entry point and nothing else. The command-line self-tests and headless
// diagnostic tools live one-per-file in src/Tests and register themselves with
// the TestRegistry, so this file does not grow when one is added.
static int runApp(int argc, char** argv)
{
	for (int argIndex= 1; argIndex < argc; ++argIndex)
	{
		if (std::string(argv[argIndex]) == "--list-tests")
		{
			TestRegistry::printAvailable();
			return 0;
		}

		int exitCode= 0;
		if (TestRegistry::tryRun(argc, argv, argIndex, exitCode))
			return exitCode;
	}

	App app;
	return app.exec(argc, argv);
}

#if defined(_WIN32) && !defined(_CONSOLE)
int WINAPI WinMain(HINSTANCE, HINSTANCE, LPSTR, int)
{
	return runApp(__argc, __argv);
}
#else
int main(int argc, char** argv)
{
	return runApp(argc, argv);
}
#endif
