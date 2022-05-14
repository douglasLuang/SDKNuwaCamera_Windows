#pragma once

#include <vector>
#include <stdlib.h>

#define DLL1_EXPORTS
#ifdef DLL1_EXPORTS
#define DLL1_API __declspec(dllexport)
#else
#define DLL1_API __declspec(dllimport)
#endif

DLL1_API int saveYUVImg(const char* const fileName, void* dataImg, const size_t size);
DLL1_API int saveCloudDot(const char* const fileName, float* cloudDot, size_t size);
