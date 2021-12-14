#pragma once

#if (defined(_MSC_VER) || defined(__CYGWIN__) || defined(__MINGW32__))
#if defined(SMARTOP_EXPORTS)
#define SMARTOP_DECLSPEC __declspec(dllexport)
#elif defined(SMARTOP_STATIC_LIBRARY)
#define SMARTOP_DECLSPEC
#else
#define SMARTOP_DECLSPEC __declspec(dllimport)
#endif
#else
#define SMARTOP_DECLSPEC
#endif