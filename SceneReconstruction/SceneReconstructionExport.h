#pragma once

#if (defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__))
	#if defined(SCENERECONSTRUCTION_EXPORTS)
	#define RECONSTRUCTION_DECLSPEC __declspec(dllexport)
	#elif defined(HIVE_STATIC_LIBRARY)
	#define RECONSTRUCTION_DECLSPEC /*sb*/
	#else
	#define RECONSTRUCTION_DECLSPEC __declspec(dllimport)
	#endif
#else
	#define RECONSTRUCTION_DECLSPEC
#endif
