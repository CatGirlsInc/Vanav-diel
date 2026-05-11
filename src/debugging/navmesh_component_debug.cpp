#include "mesh_io.h"
#include "navmesh_debug_tool.h"

#include <DetourNavMesh.h>

#include <cstdio>
#include <iostream>
#include <string>

namespace {

bool ParseVec3Strict(const std::string& s, float out[3]) {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    if (std::sscanf(s.c_str(), " %f , %f , %f ", &x, &y, &z) != 3) {
        return false;
    }
    out[0] = x;
    out[1] = y;
    out[2] = z;
    return true;
}

bool LoadMesh(const std::filesystem::path& meshPath, dtNavMesh** outNavMesh) {
    return LoadMset(meshPath, outNavMesh);
}

} // namespace

int main(int argc, char** argv) {
    debugtool::DebugArgs args;
    if (!debugtool::ParseDebugArgs(argc, argv, "mesh.bin", ParseVec3Strict, &args, std::cerr)) {
        debugtool::PrintDebugUsage(argv[0], "mesh.bin", std::cout);
        return 1;
    }

    return debugtool::RunDebugTool(args, LoadMesh, std::cout, std::cerr);
}
