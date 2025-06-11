#pragma once
#ifndef DLLEXPORT_H_INCLUDED
#define DLLEXPORT_H_INCLUDED

#include "Detour.h"
#include <cstdint>

// Cross-platform support for exporting functions
#ifdef _WIN32
#define DETOUR_API __declspec(dllexport)
#else
#define DETOUR_API __attribute__((visibility("default")))
#endif

#ifdef __cplusplus
extern "C"
{
    DETOUR_API void* allocDetour();
    DETOUR_API void freeDetour(void* ptr);    
    DETOUR_API uint32_t load(void* ptr, const char* filename);

    // Updated to match new signatures with include/exclude flags
    DETOUR_API uint32_t find_path(void* ptr, void* start, void* end, uint16_t includeFlags, uint16_t excludeFlags, float* strPath);
    DETOUR_API uint32_t find_smoothPath(void* ptr, void* start, void* end, uint16_t includeFlags, uint16_t excludeFlags, float* smoothPath);
    DETOUR_API uint32_t random_point(void* ptr, void* centerPoint, float radius, uint16_t includeFlags, uint16_t excludeFlags, float* rndPoint);
    DETOUR_API uint32_t getPolyFlags(void* ptr, void* pos, uint16_t includeFlags, uint16_t excludeFlags);

    // LOS does not take flags — unchanged
    DETOUR_API uint32_t check_los(void* ptr, void* start, void* target, float* range);
}
#endif

#endif // DLLEXPORT_H_INCLUDED
