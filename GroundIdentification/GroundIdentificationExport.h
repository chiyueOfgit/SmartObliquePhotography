#pragma once

#if (defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__))
	#if defined(GROUNDIDENTIFICATION_EXPORTS)
	#define IDENTIFICATION_DECLSPEC __declspec(dllexport)
	#elif defined(HIVE_STATIC_LIBRARY)
	#define IDENTIFICATION_DECLSPEC /*sb*/
	#else
	#define IDENTIFICATION_DECLSPEC __declspec(dllimport)
	#endif
#else
	#define IDENTIFICATION_DECLSPEC
#endif
