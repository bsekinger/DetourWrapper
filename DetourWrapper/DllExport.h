#pragma once
#ifndef DLLEXPORT_H_INCLUDED
#define DLLEXPORT_H_INCLUDED

#if defined (_MSC_VER) && (_MSC_VER >= 1921)
#pragma once
#endif

#include "Detour.h"
#include <cstdint>

#define DETOUR_API __declspec(dllexport)

#ifdef __cplusplus
extern "C"
{
    DETOUR_API void* allocDetour();
    DETOUR_API void freeDetour(void* ptr);
    DETOUR_API uint32_t load(void* ptr, const char* filename);
    DETOUR_API uint32_t find_path(void* ptr, void* start, void* end, float* strPath);
    DETOUR_API uint32_t check_los(void* ptr, void* start, void* target, float* range);
}
#endif

#endif // EXODUS_DLLEXPORT_H_INCLUDED
