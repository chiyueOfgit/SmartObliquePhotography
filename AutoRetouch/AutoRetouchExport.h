#pragma once

#if (defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__))
	#if defined(AUTORETOUCH_EXPORTS)
	#define AUTORETOUCH_DECLSPEC __declspec(dllexport)
	#elif defined(HIVE_STATIC_LIBRARY)
	#define AUTORETOUCH_DECLSPEC
	#else
	#define AUTORETOUCH_DECLSPEC __declspec(dllimport)
	#endif
#else
	#define AUTORETOUCH_DECLSPEC
#endif