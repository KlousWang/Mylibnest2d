#pragma once

#ifdef LINUX_RT
#define NEST_API
#elif defined(_WIN32) || defined(_WINDOWS)
#ifdef NEST_DLL_EXPORTS
#define NEST_API       __declspec(dllexport)
#else
#define NEST_API       __declspec(dllimport)
#endif
#else
#define NEST_API

#endif