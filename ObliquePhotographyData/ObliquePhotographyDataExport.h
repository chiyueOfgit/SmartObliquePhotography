#pragma once

#if (defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__))
	#if defined(OPDATA_EXPORTS)
	#define OPDATA_DECLSPEC __declspec(dllexport)
	#elif defined(HIVE_STATIC_LIBRARY)
	#define OPDATA_DECLSPEC
	#else
	#define OPDATA_DECLSPEC __declspec(dllimport)
	#endif
#else
	#define OPDATA_DECLSPEC
#endif
