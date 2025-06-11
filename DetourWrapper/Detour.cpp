#define _CRT_SECURE_NO_WARNINGS   // place before any #include <cstdio>
#include <cstdio>

#include "Detour.h"
#include "DetourNavMesh.h"
#include "DetourAlloc.h"
#include "DetourNavMeshQuery.h"
#include "DetourStatus.h"
#include "DetourCommon.h"
#include <iostream>
#include <glm/gtc/type_ptr.hpp>
#include <float.h>
#include <random>


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

    // Thread-local engine avoids contention if this is called from multiple threads.
    static thread_local std::mt19937 rng(std::random_device{}());
    static thread_local std::uniform_real_distribution<float> dist(0.0f, 1.0f);

    static float BetterFrand()
    {
        return dist(rng);
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

        // std::cout << "NumTiles: " << header.numTiles << std::endl;

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

    inline bool inRange(const float* v1, const float* v2, const float r, const float h)
    {
        const float dx = v2[0] - v1[0];
        const float dy = v2[1] - v1[1];
        const float dz = v2[2] - v1[2];
        return (dx * dx + dz * dz) < r * r && fabsf(dy) < h;
    }

    /// Returns the minimum of two values.
    /// @param[in]		a	Value A
    /// @param[in]		b	Value B
    /// @return The minimum of the two values.
    template<class T> inline T rcMin(T a, T b) { return a < b ? a : b; }

    /// Returns the maximum of two values.
    /// @param[in]		a	Value A
    /// @param[in]		b	Value B
    /// @return The maximum of the two values.
    template<class T> inline T rcMax(T a, T b) { return a > b ? a : b; }


    static int fixupCorridor(dtPolyRef* path, const int npath, const int maxPath,
        const dtPolyRef* visited, const int nvisited)
    {
        int furthestPath = -1;
        int furthestVisited = -1;

        // Find furthest common polygon.
        for (int i = npath - 1; i >= 0; --i)
        {
            bool found = false;
            for (int j = nvisited - 1; j >= 0; --j)
            {
                if (path[i] == visited[j])
                {
                    furthestPath = i;
                    furthestVisited = j;
                    found = true;
                }
            }
            if (found)
                break;
        }

        // If no intersection found just return current path. 
        if (furthestPath == -1 || furthestVisited == -1)
            return npath;

        // Concatenate paths.	

        // Adjust beginning of the buffer to include the visited.
        const int req = nvisited - furthestVisited;
        const int orig = rcMin(furthestPath + 1, npath);
        int size = rcMax(0, npath - orig);
        if (req + size > maxPath)
            size = maxPath - req;
        if (size)
            memmove(path + req, path + orig, size * sizeof(dtPolyRef));

        // Store visited
        for (int i = 0; i < req; ++i)
            path[i] = visited[(nvisited - 1) - i];

        return req + size;
    }

    // This function checks if the path has a small U-turn, that is,
    // a polygon further in the path is adjacent to the first polygon
    // in the path. If that happens, a shortcut is taken.
    // This can happen if the target (T) location is at tile boundary,
    // and we're (S) approaching it parallel to the tile edge.
    // The choice at the vertex can be arbitrary, 
    //  +---+---+
    //  |:::|:::|
    //  +-S-+-T-+
    //  |:::|   | <-- the step can end up in here, resulting U-turn path.
    //  +---+---+
    static int fixupShortcuts(dtPolyRef* path, int npath, dtNavMeshQuery* navQuery)
    {
        if (npath < 3)
            return npath;

        // Get connected polygons
        static const int maxNeis = 16;
        dtPolyRef neis[maxNeis];
        int nneis = 0;

        const dtMeshTile* tile = 0;
        const dtPoly* poly = 0;
        if (dtStatusFailed(navQuery->getAttachedNavMesh()->getTileAndPolyByRef(path[0], &tile, &poly)))
            return npath;

        for (unsigned int k = poly->firstLink; k != DT_NULL_LINK; k = tile->links[k].next)
        {
            const dtLink* link = &tile->links[k];
            if (link->ref != 0)
            {
                if (nneis < maxNeis)
                    neis[nneis++] = link->ref;
            }
        }

        // If any of the neighbour polygons is within the next few polygons
        // in the path, short cut to that polygon directly.
        static const int maxLookAhead = 6;
        int cut = 0;
        for (int i = dtMin(maxLookAhead, npath) - 1; i > 1 && cut == 0; i--) {
            for (int j = 0; j < nneis; j++)
            {
                if (path[i] == neis[j]) {
                    cut = i;
                    break;
                }
            }
        }
        if (cut > 1)
        {
            int offset = cut - 1;
            npath -= offset;
            for (int i = 1; i < npath; i++)
                path[i] = path[i + offset];
        }

        return npath;
    }

    static bool getSteerTarget(dtNavMeshQuery* navQuery, const float* startPos, const float* endPos,
        const float minTargetDist,
        const dtPolyRef* path, const int pathSize,
        float* steerPos, unsigned char& steerPosFlag, dtPolyRef& steerPosRef,
        float* outPoints = 0, int* outPointCount = 0)
    {
        // Find steer target.
        static const int MAX_STEER_POINTS = 3;
        float steerPath[MAX_STEER_POINTS * 3];
        unsigned char steerPathFlags[MAX_STEER_POINTS];
        dtPolyRef steerPathPolys[MAX_STEER_POINTS];
        int nsteerPath = 0;
        navQuery->findStraightPath(startPos, endPos, path, pathSize,
            steerPath, steerPathFlags, steerPathPolys, &nsteerPath, MAX_STEER_POINTS);
        if (!nsteerPath)
            return false;

        if (outPoints && outPointCount)
        {
            *outPointCount = nsteerPath;
            for (int i = 0; i < nsteerPath; ++i)
                dtVcopy(&outPoints[i * 3], &steerPath[i * 3]);
        }


        // Find vertex far enough to steer to.
        int ns = 0;
        while (ns < nsteerPath)
        {
            // Stop at Off-Mesh link or when point is further than slop away.
            if ((steerPathFlags[ns] & DT_STRAIGHTPATH_OFFMESH_CONNECTION) ||
                !inRange(&steerPath[ns * 3], startPos, minTargetDist, 1000.0f))
                break;
            ns++;
        }
        // Failed to find good point to steer to.
        if (ns >= nsteerPath)
            return false;

        dtVcopy(steerPos, &steerPath[ns * 3]);
        steerPos[1] = startPos[1];
        steerPosFlag = steerPathFlags[ns];
        steerPosRef = steerPathPolys[ns];

        return true;
    }

    static void dumpDetourTile(const dtMeshTile* tile)
    {
        if (!tile || !tile->header) return;

        puts("---- tile dump ----");
        const dtPoly* polys = tile->polys;
        const int     pcnt = tile->header->polyCount;

        for (int p = 0; p < pcnt; ++p)
        {
            const dtPoly& poly = polys[p];
            //if (poly.type == DT_POLYTYPE_OFFMESH_CONNECTION) continue;

            unsigned char  area = poly.getArea();  // or poly.area on older forks
            unsigned short flags = poly.flags;

            printf("poly[%3d] area=%3u  flags=0x%04X\n",
                p, static_cast<unsigned>(area), static_cast<unsigned>(flags));
        }
    }

    uint32_t detour::random_point(const glm::vec3& centerPoint, float radius, uint16_t includeFlags, uint16_t excludeFlags, float* rndPoint)
    {
        m_dtNavMeshQuery->init(m_dtNavMesh.get(), 65535);

        const float* centerPtr = glm::value_ptr(centerPoint);

        const glm::vec3 extents(2.0f, 50.0f, 2.0f);
        const float* halfExtents = glm::value_ptr(extents);

        dtPolyRef centerRef, randomRef;

        float nearestPt[3];

        dtQueryFilter filter;
        filter.setIncludeFlags(includeFlags);
        filter.setExcludeFlags(excludeFlags);

        filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
        filter.setAreaCost(SAMPLE_POLYAREA_WATER, 1.5f);
        filter.setAreaCost(SAMPLE_POLYAREA_MUD, 3.0f);
        filter.setAreaCost(SAMPLE_POLYAREA_LAVA, 100.0f);  // Basically avoid

        dtStatus status = m_dtNavMeshQuery->findNearestPoly(centerPtr, halfExtents, &filter, &centerRef, nearestPt);
        if (dtStatusFailed(status))
        {
            // std::cout << "Could not find valid center poly! " << "Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->findRandomPointAroundCircle(centerRef, centerPtr, radius, &filter, BetterFrand, &randomRef, rndPoint);
        if (dtStatusFailed(status))
        {
            // std::cout << "Could not find random point within radius! " << "Status: " << status << std::endl;
            return 0;
        }

        return 1;
    }
 
    uint32_t detour::find_path(const glm::vec3& startPoint, const glm::vec3& endPoint, uint16_t includeFlags, uint16_t excludeFlags, float* strPath)
    {
         m_dtNavMeshQuery->init(m_dtNavMesh.get(), 65535);

        const float* startptr = glm::value_ptr(startPoint);
        const float* endptr = glm::value_ptr(endPoint);
        
        const glm::vec3 extents(2.0f, 50.0f, 2.0f);
        const float* halfExtents = glm::value_ptr(extents);

        dtPolyRef startRef, endRef;
        
        float startPt[3];
        float endPt[3];
        
        dtQueryFilter filter;
        filter.setIncludeFlags(includeFlags);
        filter.setExcludeFlags(excludeFlags);

        filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
        filter.setAreaCost(SAMPLE_POLYAREA_WATER, 1.5f);
        filter.setAreaCost(SAMPLE_POLYAREA_MUD, 3.0f);
        filter.setAreaCost(SAMPLE_POLYAREA_LAVA, 100.0f);  // Basically avoid

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
            // std::cout << "Could not find valid start poly! " << "Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->findNearestPoly(endptr, halfExtents, &filter, &endRef, endPt);
        if (dtStatusFailed(status))
        {
            // std::cout << "Could not find valid end poly! " << "Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->findPath(startRef, endRef, startPt, endPt, &filter, path, &pathCount, MAX_POLYS);
        if (dtStatusFailed(status))
        {
            // std::cout << "Could not find valid path! " << "Status: " << status << std::endl;
            return 0;
        }

        if (pathCount > 0)
        {
            status = m_dtNavMeshQuery->findStraightPath(startPt, endPt, path, pathCount, straightPath, strPathFlags, strPathPolys, &strPathCount, MAX_POLYS);
            if (dtStatusFailed(status))
            {
                // std::cout << "Could not find valid straight path! " << "Status: " << status << std::endl;
                return 0;
            }
        }

        for (int i = 0; i < MAX_POLYS * 3; ++i)
        {
            strPath[i] = straightPath[i];
        }
        return strPathCount;
    }

    uint32_t detour::find_smoothPath(const glm::vec3& startPoint, const glm::vec3& endPoint, uint16_t includeFlags, uint16_t excludeFlags, float* smoothPath)
    {
        m_dtNavMeshQuery->init(m_dtNavMesh.get(), 65535);

        const float* startPtr = glm::value_ptr(startPoint);
        const float* endPtr = glm::value_ptr(endPoint);

        const glm::vec3 extents(2.0f, 50.0f, 2.0f);
        const float* halfExtents = glm::value_ptr(extents);

        dtPolyRef startRef, endRef;

        float nearestStartPos[3];
        float nearestEndPos[3];

        dtQueryFilter filter;
        filter.setIncludeFlags(includeFlags);
        filter.setExcludeFlags(excludeFlags);

        filter.setAreaCost(SAMPLE_POLYAREA_GROUND, 1.0f);
        filter.setAreaCost(SAMPLE_POLYAREA_WATER, 1.5f);
        filter.setAreaCost(SAMPLE_POLYAREA_MUD, 3.0f);
        filter.setAreaCost(SAMPLE_POLYAREA_LAVA, 100.0f);  // Basically avoid

        dtPolyRef path[MAX_POLYS];
        dtStatus status = 0;
        int pathCount = 0;

        // Find the nearest polygons to the start and end points
        status = m_dtNavMeshQuery->findNearestPoly(startPtr, halfExtents, &filter, &startRef, nearestStartPos);
        if (dtStatusFailed(status))
        {
            // std::cout << "Could not find valid start poly! Status: " << status << std::endl;
            return 0;
        }

        status = m_dtNavMeshQuery->findNearestPoly(endPtr, halfExtents, &filter, &endRef, nearestEndPos);
        if (dtStatusFailed(status))
        {
            // std::cout << "Could not find valid end poly! Status: " << status << std::endl;
            return 0;
        }

        // Find a path between the start and end polygons
        status = m_dtNavMeshQuery->findPath(startRef, endRef, nearestStartPos, nearestEndPos, &filter, path, &pathCount, MAX_POLYS);
        if (dtStatusFailed(status) || pathCount == 0)
        {
            // std::cout << "Could not find valid path! Status: " << status << std::endl;
            return 0;
        }

        // Refining the path using moveAlongSurface for a smoother path
        float iterPos[3], targetPos[3];
        m_dtNavMeshQuery->closestPointOnPoly(startRef, nearestStartPos, iterPos, nullptr);
        m_dtNavMeshQuery->closestPointOnPoly(path[pathCount - 1], nearestEndPos, targetPos, nullptr);

        const float STEP_SIZE = 2.0f;
        const float SLOP = 0.01f;

        int smoothPathCount = 0;
        dtVcopy(&smoothPath[smoothPathCount * 3], iterPos);
        smoothPathCount++;

        while (pathCount > 0 && smoothPathCount < MAX_SMOOTH)
        {
            float steerPos[3];
            unsigned char steerPosFlag;
            dtPolyRef steerPosRef;

            // Custom function to get the steer target, not provided in the original code snippet
            if (!getSteerTarget(m_dtNavMeshQuery.get(), iterPos, targetPos, SLOP, path, pathCount, steerPos, steerPosFlag, steerPosRef))
                break;

            bool atEnd = steerPosFlag & DT_STRAIGHTPATH_END;
            bool offMeshConnection = steerPosFlag & DT_STRAIGHTPATH_OFFMESH_CONNECTION;

            float delta[3], len;
            dtVsub(delta, steerPos, iterPos);
            len = sqrtf(dtVdot(delta, delta));
            if ((atEnd || offMeshConnection) && len < STEP_SIZE)
                len = 1;
            else
                len = STEP_SIZE / len;

            float moveTgt[3];
            dtVmad(moveTgt, iterPos, delta, len);

            // Move along surface
            float result[3];
            dtPolyRef visited[16];
            int nvisited = 0;
            m_dtNavMeshQuery->moveAlongSurface(path[0], iterPos, moveTgt, &filter, result, visited, &nvisited, 16);

            pathCount = fixupCorridor(path, pathCount, MAX_POLYS, visited, nvisited);
            pathCount = fixupShortcuts(path, pathCount, m_dtNavMeshQuery.get());

            float h = 0;
            m_dtNavMeshQuery->getPolyHeight(path[0], result, &h);
            result[1] = h;
            dtVcopy(iterPos, result);

            if (smoothPathCount < MAX_SMOOTH)
            {
                dtVcopy(&smoothPath[smoothPathCount * 3], iterPos);
                smoothPathCount++;
            }
        }

        return smoothPathCount;
    }


    uint32_t detour::check_los(const glm::vec3& start, const glm::vec3& target, float* range)
    {
        m_dtNavMeshQuery->init(m_dtNavMesh.get(), 65535);
        float distance = glm::distance(start, target);
        const float* startptr = glm::value_ptr(start);
        const float* targetptr = glm::value_ptr(target);

        const glm::vec3 extents(2.0f, 4.0f, 2.0f);
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

    uint32_t eqoa::detour::getPolyFlags(const glm::vec3& pos, uint16_t includeFlags, uint16_t excludeFlags)
    {        
        m_dtNavMeshQuery->init(m_dtNavMesh.get(), 65535);
        
        const glm::vec3 extents(0.1f,30.0f, 0.1f);
        const float* halfExtents = glm::value_ptr(extents);
        
        dtQueryFilter filter;
        filter.setIncludeFlags(includeFlags);
        filter.setExcludeFlags(excludeFlags);
        
        dtPolyRef ref = 0;
        float nearestPt[3];
        dtStatus status = m_dtNavMeshQuery->findNearestPoly(glm::value_ptr(pos), halfExtents, &filter, &ref, nearestPt);

        if (dtStatusFailed(status) || !ref)
            return UINT32_MAX;

        //// ** DEBUG: dump the entire tile that this poly belongs to DELETE AFTER TESTING! **
        //{
        //    const dtMeshTile* tile = nullptr;
        //    const dtPoly* poly = nullptr;
        //    if (m_dtNavMesh->getTileAndPolyByRef(ref, &tile, &poly) == DT_SUCCESS)
        //    {
        //        if (poly)
        //        {
        //            printf("----- Dumping poly ref 0x%016llX -----\n", (unsigned long long)ref);
        //            printf(" flags = 0x%02X, verts = %d\n",
        //                poly->flags,
        //                poly->vertCount);
        //            for (int i = 0; i < poly->vertCount; ++i)
        //            {
        //                // each poly->verts[i] is an index into tile->verts (flat float array)
        //                unsigned int vi = poly->verts[i];
        //                const float* v = &tile->verts[3 * vi];
        //                printf("   vert[%d] (idx=%d):  %.3f, %.3f, %.3f\n",
        //                    i, vi, v[0], v[1], v[2]);
        //            }
        //            puts("--------------------------------------");
        //        }
        //        else
        //        {
        //            puts("Failed to locate poly pointer");
        //        }
        //    }
        //    else
        //    { 
        //        puts("Failed to locate tile from ref");
        //    }
        //}

        unsigned short flags = 0;
        m_dtNavMesh->getPolyFlags(ref, &flags);

        // ► Return the full flag bits now:
        return flags;
    }

     void detour::unload()
    {
        //Both mesh and query will be freed when detour instance is destroyed
        //do we need anything here?
    }
}


