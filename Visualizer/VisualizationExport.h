#pragma once

#if (defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__))
	#if defined(VISUALIZATION_EXPORTS)
	#define VISUALIZATION_DECLSPEC __declspec(dllexport)
	#elif defined(HIVE_STATIC_LIBRARY)
	#define VISUALIZATION_DECLSPEC
	#else
	#define VISUALIZATION_DECLSPEC __declspec(dllimport)
	#endif
#else
	#define VISUALIZATION_DECLSPEC
#endif