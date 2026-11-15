#pragma once

#include "ThreadUtils.h"

// The instrumented profiler's macros. Application code uses these and never
// the profiler's own, so a build without MIKANTRACK_TRACY includes none of
// its headers and every macro here is empty, the thread name aside.
//
//   MIKAN_PROFILE_FUNCTION()               a zone named for the function, to the end of the scope
//   MIKAN_PROFILE_SCOPE("name")            a named zone, to the end of the scope
//   MIKAN_PROFILE_SCOPE_TEXT("name", t, n) the same, carrying n characters of text t (a model's file name)
//   MIKAN_PROFILE_FRAME()                  the end of a frame of the main loop
//   MIKAN_PROFILE_FRAME_NAMED("name")      the end of a frame of another loop
//   MIKAN_PROFILE_PLOT("name", value)      one sample of a plotted number
//   MIKAN_PROFILE_THREAD("name")           names the calling thread, for the debugger too
//
// Names are string literals: the profiler keeps the pointer. Several zones
// may share a scope.

#ifdef MIKANTRACK_TRACY

#include "tracy/Tracy.hpp"

#define MIKAN_PROFILE_ZONE_VAR TracyConcat(mikanProfileZone, TracyLine)
#define MIKAN_PROFILE_FUNCTION() ZoneNamed(MIKAN_PROFILE_ZONE_VAR, true)
#define MIKAN_PROFILE_SCOPE(name) ZoneNamedN(MIKAN_PROFILE_ZONE_VAR, name, true)
#define MIKAN_PROFILE_SCOPE_TEXT(name, text, length)                                                                   \
	ZoneNamedN(MIKAN_PROFILE_ZONE_VAR, name, true);                                                                    \
	MIKAN_PROFILE_ZONE_VAR.Text(text, length)
#define MIKAN_PROFILE_FRAME() FrameMark
#define MIKAN_PROFILE_FRAME_NAMED(name) FrameMarkNamed(name)
#define MIKAN_PROFILE_PLOT(name, value) TracyPlot(name, value)
#define MIKAN_PROFILE_THREAD(name)                                                                                     \
	do                                                                                                                 \
	{                                                                                                                  \
		ThreadUtils::setCurrentThreadName(name);                                                                       \
		tracy::SetThreadName(name);                                                                                    \
	} while (false)

#else

#define MIKAN_PROFILE_FUNCTION()
#define MIKAN_PROFILE_SCOPE(name)
#define MIKAN_PROFILE_SCOPE_TEXT(name, text, length)
#define MIKAN_PROFILE_FRAME()
#define MIKAN_PROFILE_FRAME_NAMED(name)
#define MIKAN_PROFILE_PLOT(name, value)
#define MIKAN_PROFILE_THREAD(name) ThreadUtils::setCurrentThreadName(name)

#endif
