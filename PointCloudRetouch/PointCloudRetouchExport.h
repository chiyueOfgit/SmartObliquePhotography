#pragma once

#if (defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__))
	#if defined(POINTCLOUDRETOUCH_EXPORTS)
	#define RETOUCH_DECLSPEC __declspec(dllexport)
	#elif defined(HIVE_STATIC_LIBRARY)
	#define RETOUCH_DECLSPEC
	#else
	#define RETOUCH_DECLSPEC __declspec(dllimport)
	#endif
#else
	#define RETOUCH_DECLSPEC
#endif