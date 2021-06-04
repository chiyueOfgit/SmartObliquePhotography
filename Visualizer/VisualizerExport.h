#pragma once

#if (defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__))
	#if defined(VISUALIZER_EXPORTS)
	#define VISUALIZER_DECLSPEC __declspec(dllexport)
	#elif defined(HIVE_STATIC_LIBRARY)
	#define VISUALIZER_DECLSPEC
	#else
	#define VISUALIZER_DECLSPEC __declspec(dllimport)
	#endif
#else
	#define VISUALIZER_DECLSPEC
#endif