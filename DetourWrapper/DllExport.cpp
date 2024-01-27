#include <glm/glm.hpp>
#include "detour.h"
#include "dllexport.h"

DETOUR_API void* allocDetour()
{
    eqoa::detour* detour = new eqoa::detour();
    return static_cast<void*>(detour);
}

DETOUR_API void freeDetour(void* ptr)
{
    eqoa::detour* detour = static_cast<eqoa::detour*>(ptr);
    delete detour;
}

DETOUR_API uint32_t load(void* ptr, const char* filename)
{
    eqoa::detour* detour = static_cast<eqoa::detour*>(ptr);
    return detour->load(std::string{ filename });
}

DETOUR_API uint32_t find_path(void* ptr, void* start, void* end, float* strPath)
{
    eqoa::detour* detour = static_cast<eqoa::detour*>(ptr);
    return detour->find_path(*static_cast<glm::vec3*>(start), *static_cast<glm::vec3*>(end), strPath);
}

DETOUR_API uint32_t random_roam(void* ptr, void* start, float* strPath)
{
    eqoa::detour* detour = static_cast<eqoa::detour*>(ptr);
    return detour->random_roam(*static_cast<glm::vec3*>(start), strPath);
}

DETOUR_API uint32_t check_los(void* ptr, void* start, void* target, float* range)
{
    eqoa::detour* detour = static_cast<eqoa::detour*>(ptr);
    return detour->check_los(*static_cast<glm::vec3*>(start), *static_cast<glm::vec3*>(target), range);
}

DETOUR_API uint32_t random_point(void* ptr, void* centerPoint, float radius, float* rndPoint)
{
    eqoa::detour* detour = static_cast<eqoa::detour*>(ptr);
    return detour->random_point(*static_cast<const glm::vec3*>(centerPoint), radius, rndPoint);
}
