#include "navmesh_debug_tool.h"
#include "routing/zone_route.h"

#include <DetourNavMesh.h>

#include <filesystem>
#include <iostream>
#include <string>

namespace {

bool ParseVec3Strict(const std::string& s, float out[3]) {
    return ParseVec3(s, out);
}

bool LoadMesh(const std::filesystem::path& meshPath, dtNavMesh** outNavMesh) {
    return BuildNavMeshFromFfxiBin(meshPath, outNavMesh);
}

} // namespace

int main(int argc, char** argv) {
    debugtool::DebugArgs args;
    if (!debugtool::ParseDebugArgs(argc, argv, "zone.bin", ParseVec3Strict, &args, std::cerr)) {
        debugtool::PrintDebugUsage(argv[0], "zone.bin", std::cout);
        return 1;
    }

    return debugtool::RunDebugTool(args, LoadMesh, std::cout, std::cerr);
}
