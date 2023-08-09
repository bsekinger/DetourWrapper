//DetourWrapper.h
#pragma once
#ifndef DETOURWRAPPER_H
#define DETOURWRAPPER_H
#endif

using namespace System;

#include "DetourNavMesh.h"
#include "DetourAlloc.h"

namespace DetourWrapper
{
     // Class containing methods to load a mesh and other functions
    public ref class MeshLoader
    {
    public:
        // Function to load a mesh from the file and return the serialized data
        static void* LoadMesh(System::String^ filePath);

        // Function to free the dtNavMesh memory
        static void FreeMesh(void* navMeshIntPtr);

    };
}




