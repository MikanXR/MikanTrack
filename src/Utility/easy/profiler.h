#pragma once

// The parent folder by path: this file shares the wrapper's name but for its case
#include "../Profiler.h"

// The easy_profiler macros, which MikanTrack does not use, over the
// profiler's own zones. Lets code copied from MikanXR keep its EASY_*
// instrumentation lines unmodified.
#define EASY_FUNCTION(...) MIKAN_PROFILE_FUNCTION()
#define EASY_BLOCK(name, ...) MIKAN_PROFILE_SCOPE(name)
#define EASY_END_BLOCK
#define EASY_NONSCOPED_BLOCK(...)
#define EASY_MAIN_THREAD
#define EASY_THREAD(...)
#define EASY_PROFILER_ENABLE
