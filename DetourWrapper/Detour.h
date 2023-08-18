#ifndef DETOUR_H_INCLUDED
#define DETOUR_H_INCLUDED

#if defined (_MSC_VER) && (_MSC_VER >= 1921)
#pragma once
#endif

#include <cstdint>
#include <fstream>
#include <glm/vec3.hpp>
#include <memory>
#include <vector>

#include "DetourNavMesh.h"
#include "DetourNavMeshQuery.h"

#define MAX_POLYS 256

namespace eqoa
{
    class detour
    {
    public:
        detour();
        ~detour();
        uint32_t load(const std::string& filePath);
        uint32_t find_path(const glm::vec3& startPoint, const glm::vec3& endPoint, float* strPath);
        uint32_t check_los(const glm::vec3& start, const glm::vec3& target, float* range);
    private:
        void unload();
        std::unique_ptr<dtNavMesh> m_dtNavMesh;
        std::unique_ptr<dtNavMeshQuery> m_dtNavMeshQuery;
    };
}

#endif // DETOUR_H_INCLUDED




