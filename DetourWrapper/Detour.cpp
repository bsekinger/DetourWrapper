#include "Detour.h"
#include "DetourNavMesh.h"
#include "DetourAlloc.h"
#include "DetourNavMeshQuery.h"
#include "DetourStatus.h"
#include "DetourCommon.h"
#include <iostream>
#include <glm/gtc/type_ptr.hpp>
#include <float.h>


namespace eqoa
{
    static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T';
    static const int NAVMESHSET_VERSION = 1;

    struct NavMeshSetHeader
    {
        int magic;
        int version;
        int numTiles;
        dtNavMeshParams params;
    };

    struct NavMeshTileHeader
    {
        dtTileRef tileRef;
        int dataSize;
    };

    // Returns a random number [0..1]
    static float frand()
    {
        //	return ((float)(rand() & 0xffff)/(float)0xffff);
        return (float)rand() / (float)RAND_MAX;
    }

    dtNavMesh* LoadMeshFile(const std::string& filePath)
    {
        FILE* file = fopen(filePath.c_str(), "rb");
        if (!file)
        {
            std::cout << "File Not Found!" << std::endl;
            return nullptr;
        }

        NavMeshSetHeader header;
        size_t readLen = fread(&header, sizeof(NavMeshSetHeader), 1, file);
        if (readLen != 1)
        {
            fclose(file);
            return nullptr;
        }

        if (header.magic != NAVMESHSET_MAGIC)
        {
            fclose(file);
            return nullptr;
        }

        if (header.version != NAVMESHSET_VERSION)
        {
            fclose(file);
            return nullptr;
        }

        std::cout << "NumTiles: " << header.numTiles << std::endl;

        dtNavMesh* mesh = dtAllocNavMesh();
        if (!mesh)
        {
            fclose(file);
            return nullptr;
        }
        dtStatus status = mesh->init(&header.params);
        if (dtStatusFailed(status))
        {
            fclose(file);
            return nullptr;
        }

        for (int i = 0; i < header.numTiles; ++i)
        {
            NavMeshTileHeader tileHeader;
            readLen = fread(&tileHeader, sizeof(tileHeader), 1, file);
            if (readLen != 1)
            {
                fclose(file);
                return nullptr;
            }

            if (!tileHeader.tileRef || !tileHeader.dataSize)
            {
                break;
            }

            unsigned char* data = (unsigned char*)dtAlloc(tileHeader.dataSize, DT_ALLOC_PERM);
            if (!data)
            {
                break;
            }

            memset(data, 0, tileHeader.dataSize);
            readLen = fread(data, tileHeader.dataSize, 1, file);
            if (readLen != 1)
            {
                dtFree(data);
                fclose(file);
                return nullptr;
            }

            mesh->addTile(data, tileHeader.dataSize, DT_TILE_FREE_DATA, tileHeader.tileRef, 0);
        }

        fclose(file);
        return mesh;
    }

    detour::detour()
    {
        m_dtNavMesh = std::make_unique<dtNavMesh>();
        m_dtNavMeshQuery = std::make_unique<dtNavMeshQuery>();
    }

    detour::~detour()
    {
        // m_dtNavMesh and m_dtNavMeshQuery will be automatically cleaned up
        // when the detour object is destructed. Do we need this?
    }

    uint32_t detour::load(const std::string& filePath)
    {
        std::unique_ptr<dtNavMesh> loadedMesh(LoadMeshFile(filePath));

        if (loadedMesh)
        {
            m_dtNavMesh = std::move(loadedMesh);
            return 1;
        }

        return 0;
     }

    uint32_t detour::random_roam(const glm::vec3& startPoint, float* strPath)
    {
        m_dtNavMeshQuery->init(m_dtNavMesh.get(), 65535);

        const float* startptr = glm::value_ptr(startPoint);

        glm::vec3 extents(2.0f, 4.0f, 2.0f);
        const float* halfExtents = glm::value_ptr(extents);

        dtPolyRef startRef, rndRef;

        float startPt[3];
        float rndPt[3];

        dtQueryFilter filter;
        filter.setIncludeFlags(0xffff);
        filter.setExcludeFlags(0);

        dtPolyRef path[MAX_POLYS];
        dtStatus status = 0;
        int pathCount = 0;

        float straightPath[MAX_POLYS * 3]{};
        unsigned char strPathFlags[MAX_POLYS];
        dtPolyRef strPathPolys[MAX_POLYS];

        int strPathCount = 0;

        status = m_dtNavMeshQuery->findNearestPoly(startptr, halfExtents, &filter, &startRef, startPt);
        if (dtStatusFailed(status))
        {
            std::cout << "Could not find valid start poly! " << "Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->findRandomPointAroundCircle(startRef, startptr, 20.0f, &filter, frand, &rndRef, rndPt);
        if (dtStatusFailed(status))
        {
            std::cout << "Could not find random point around circle! " << "Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->findPath(startRef, rndRef, startPt, rndPt, &filter, path, &pathCount, MAX_POLYS);
        if (dtStatusFailed(status))
        {
            std::cout << "Could not find valid path! " << "Status: " << status << std::endl;
            return 0;
        }

        if (pathCount > 0)
        {
            status = m_dtNavMeshQuery->findStraightPath(startPt, rndPt, path, pathCount, straightPath, strPathFlags, strPathPolys, &strPathCount, MAX_POLYS);
            if (dtStatusFailed(status))
            {
                std::cout << "Could not find valid straight path! " << "Status: " << status << std::endl;
                return 0;
            }
        }

        for (int i = 0; i < MAX_POLYS * 3; ++i)
        {
            strPath[i] = straightPath[i];
        }
        return strPathCount;        
    }

    uint32_t detour::find_path(const glm::vec3& startPoint, const glm::vec3& endPoint, float* strPath)
    {
         m_dtNavMeshQuery->init(m_dtNavMesh.get(), 65535);

        const float* startptr = glm::value_ptr(startPoint);
        const float* endptr = glm::value_ptr(endPoint);
        
        glm::vec3 extents(2.0f, 4.0f, 2.0f);
        const float* halfExtents = glm::value_ptr(extents);

        dtPolyRef startRef, endRef;
        
        float startPt[3];
        float endPt[3];
        
        dtQueryFilter filter;
        filter.setIncludeFlags(0xffff);
        filter.setExcludeFlags(0);

        dtPolyRef path[MAX_POLYS];
        dtStatus status = 0;
        int pathCount = 0;

        float straightPath[MAX_POLYS * 3]{};
        unsigned char strPathFlags[MAX_POLYS];
        dtPolyRef strPathPolys[MAX_POLYS];

        int strPathCount = 0;

        status = m_dtNavMeshQuery->findNearestPoly(startptr, halfExtents, &filter, &startRef, startPt);
        if (dtStatusFailed(status))
        {
            std::cout << "Could not find valid start poly! " << "Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->findNearestPoly(endptr, halfExtents, &filter, &endRef, endPt);
        if (dtStatusFailed(status))
        {
            std::cout << "Could not find valid end poly! " << "Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->findPath(startRef, endRef, startPt, endPt, &filter, path, &pathCount, MAX_POLYS);
        if (dtStatusFailed(status))
        {
            std::cout << "Could not find valid path! " << "Status: " << status << std::endl;
            return 0;
        }

        if (pathCount > 0)
        {
            status = m_dtNavMeshQuery->findStraightPath(startPt, endPt, path, pathCount, straightPath, strPathFlags, strPathPolys, &strPathCount, MAX_POLYS);
            if (dtStatusFailed(status))
            {
                std::cout << "Could not find valid straight path! " << "Status: " << status << std::endl;
                return 0;
            }
        }

        for (int i = 0; i < MAX_POLYS * 3; ++i)
        {
            strPath[i] = straightPath[i];
        }
        return strPathCount;
    }

    uint32_t detour::check_los(const glm::vec3& start, const glm::vec3& target, float* range)
    {
        m_dtNavMeshQuery->init(m_dtNavMesh.get(), 65535);
        float distance = glm::distance(start, target);
        const float* startptr = glm::value_ptr(start);
        const float* targetptr = glm::value_ptr(target);

        glm::vec3 extents(2.0f, 4.0f, 2.0f);
        const float* halfExtents = glm::value_ptr(extents);

        float startPt[3]{ start.x, start.y, start.z };
        float targetPt[3]{ target.x, target.y, target.z };

        dtStatus status = 0;
        dtPolyRef startRef;

        dtQueryFilter filter;
        filter.setIncludeFlags(0xffff);
        filter.setExcludeFlags(0);

        dtRaycastHit hit;
        dtPolyRef prevRef = 0;
        const unsigned int options = 1;

        float t;
        float hitNormal;
        dtPolyRef path;
        int pathCount;
        const int maxPath = 256;

        if (distance > *range)
        {
            //std::cout << "target is out of range. Distance: " << distance << " range: " << *range << std::endl;
            return 1;
        }

        status = m_dtNavMeshQuery->findNearestPoly(startptr, halfExtents, &filter, &startRef, startPt);
        if (dtStatusFailed(status))
        {
            //std::cout << "Could not find valid start poly! " << "Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->raycast(startRef, startPt, targetPt, &filter, &t, &hitNormal, &path, &pathCount, maxPath);
        if (dtStatusFailed(status))
        {
            return 0;
        }

        if (t >= 3e+38)
        {
            //std::cout << "LoS Success!" << std::endl;
            return 5;
        }

        //std::cout << "LoS Failed!" << std::endl;
        return 2;
    }

     void detour::unload()
    {
        //Both mesh and query will be freed when detour instance is destroyed
        //do we need anything here?
    }
}


