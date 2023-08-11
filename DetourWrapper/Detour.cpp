#include "Detour.h"
#include "DetourNavMesh.h"
#include "DetourAlloc.h"
#include <iostream>

namespace eqoa
{
    static const int NAVMESHSET_MAGIC = 'M' << 24 | 'S' << 16 | 'E' << 8 | 'T'; //'MSET';
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

    dtNavMesh* LoadMeshFile(const std::string& filePath)
    {
        // Open the binary file for reading
        FILE* file = fopen(filePath.c_str(), "rb");
        if (!file)
        {
            std::cout << "File Not Found!" << std::endl;
            return nullptr;
        }

        // Read header.
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

        // Read and load tiles.
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

     // MeshLoader LoadMesh function
    uint32_t detour::load(const std::string& filePath)
    {
        std::unique_ptr<dtNavMesh> loadedMesh(LoadMeshFile(filePath));

        if (loadedMesh)
        {
            m_dtNavMesh = std::move(loadedMesh); // Transfer ownership            
            std::cout << "LoadMesh complete!" << std::endl;
            std::cout << "Mesh pointer: " << m_dtNavMesh << std::endl;

            return 1; // success
        }

        return 0;
     }

    // MeshLoader FreeMesh function
    void detour::unload()
    {
        //Both mesh and query will be freed when detour instance is destroyed
        //do we need anything here?
    }
}


