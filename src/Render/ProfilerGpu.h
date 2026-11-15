#pragma once

// The profiler's OpenGL zones, timed on the GPU by timer queries. Included
// after the GL loader, by the files that draw. Empty without
// MIKANTRACK_TRACY.
//
//   MIKAN_PROFILE_GPU_CONTEXT()     once, on the thread that owns the GL context, after it is current
//   MIKAN_PROFILE_GPU_SCOPE("name") a GPU zone to the end of the scope
//   MIKAN_PROFILE_GPU_COLLECT()     once a frame after the buffer swap, reads the finished queries back

#ifdef MIKANTRACK_TRACY

#include <GL/glew.h>

#include "tracy/Tracy.hpp"
#include "tracy/TracyOpenGL.hpp"

#define MIKAN_PROFILE_GPU_CONTEXT() TracyGpuContext
#define MIKAN_PROFILE_GPU_SCOPE(name) TracyGpuNamedZone(TracyConcat(mikanProfileGpuZone, TracyLine), name, true)
#define MIKAN_PROFILE_GPU_COLLECT() TracyGpuCollect

#else

#define MIKAN_PROFILE_GPU_CONTEXT()
#define MIKAN_PROFILE_GPU_SCOPE(name)
#define MIKAN_PROFILE_GPU_COLLECT()

#endif
