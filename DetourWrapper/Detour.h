#ifndef DETOUR_H_INCLUDED
#define DETOUR_H_INCLUDED

#if defined (_MSC_VER) && (_MSC_VER >= 1921)
#pragma once
#endif

#include <cstdint>
#include <fstream>
#include <glm/glm.hpp>
#include <memory>
#include <vector>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#define MAX_POLYS 256
#define MAX_SMOOTH 2048

enum SamplePolyAreas
{
    SAMPLE_POLYAREA_GROUND = 0,
    SAMPLE_POLYAREA_WATER = 1,
    SAMPLE_POLYAREA_MUD = 2,
    SAMPLE_POLYAREA_LAVA = 3
};

enum SamplePolyFlags
{
    SAMPLE_POLYFLAGS_WALK = 0x01,  // Normal ground
    SAMPLE_POLYFLAGS_SWIM = 0x02,  // Generic swim
    SAMPLE_POLYFLAGS_WATER = 0x04,  // Swim: low cost
    SAMPLE_POLYFLAGS_MUD = 0x08,  // Swim: medium cost
    SAMPLE_POLYFLAGS_LAVA = 0x10,  // Swim: high cost
    SAMPLE_POLYFLAGS_ALL = 0xffff
};

namespace eqoa
{
    class  detour
    {
    public:
        detour();
        ~detour();
        uint32_t load(const std::string& filePath);
        uint32_t find_path(const glm::vec3& startPoint, const glm::vec3& endPoint, uint16_t includeFlags, uint16_t excludeFlags, float* strPath);
        uint32_t find_smoothPath(const glm::vec3& startPoint, const glm::vec3& endPoint, uint16_t includeFlags, uint16_t excludeFlags, float* smoothPath);
        uint32_t random_point(const glm::vec3& centerPoint, float radius, uint16_t includeFlags, uint16_t excludeFlags, float* rndPoint);
        uint32_t check_los(const glm::vec3& start, const glm::vec3& target, float* range);
        uint32_t getPolyFlags(const glm::vec3& pos, uint16_t includeFlags, uint16_t excludeFlags);

    private:
        void unload();
        std::unique_ptr<dtNavMesh> m_dtNavMesh;
        std::unique_ptr<dtNavMeshQuery> m_dtNavMeshQuery;
    };
}

#endif // DETOUR_H_INCLUDED




