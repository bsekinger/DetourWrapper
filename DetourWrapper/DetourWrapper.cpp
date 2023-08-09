#include "DetourWrapper.h"
#include "DetourNavMesh.h"
#include "DetourAlloc.h"
#include <string.h>
#include <string>
#include <stdio.h>
#include <vcclr.h>
#include <iostream>

namespace DetourWrapper
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

     // MeshLoader LoadMesh function
    void* MeshLoader::LoadMesh(System::String^ filePath)
    {
        const char* nativeFilePath = (const char*)(void*)System::Runtime::InteropServices::Marshal::StringToHGlobalAnsi(filePath);

        // Load the mesh file and create the dtNavMesh
        dtNavMesh* navMesh = LoadMeshFile(nativeFilePath);
        
        void* meshPtr = nullptr;

        if (navMesh)
        {
            meshPtr = navMesh;
            std::cout << "LoadMesh complete!" << std::endl;
            std::cout << "Mesh pointer: " << meshPtr << std::endl;
        }

        return meshPtr;
     }

    // MeshLoader FreeMesh function
    void MeshLoader::FreeMesh(void* navMeshVoidPtr)
    {
        dtNavMesh* navMeshPointer = nullptr;
        navMeshPointer = reinterpret_cast<dtNavMesh*>(navMeshVoidPtr);

        if (navMeshPointer)   
        {       
            dtFreeNavMesh(navMeshPointer);
            navMeshPointer = nullptr;
            std::cout << "navMesh freed!" << std::endl;
        }
    }
}
