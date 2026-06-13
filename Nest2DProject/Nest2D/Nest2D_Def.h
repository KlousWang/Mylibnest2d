#pragma once

#ifdef LINUX_RT

#define NEST2D_API

#elif defined(_WIN32) || defined(_WINDOWS)

#ifdef NEST2D_EXPORTS
#define NEST2D_API __declspec(dllexport)
#else
#define NEST2D_API __declspec(dllimport)
#endif

#else

#define NEST2D_API

#endif