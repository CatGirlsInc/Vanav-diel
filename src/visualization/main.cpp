#include "pathfinding/routing/zone_route.h"
#include "viz.h"

#include <DetourNavMesh.h>
#include <DetourAlloc.h>

#include <filesystem>
#include <iostream>
#include <vector>

int main(int argc, char** argv) {
    const AppArgs args = ParseArgs(argc, argv);

    if (args.infoOnly || args.queryMode || args.worldQuery) {
        std::cerr << "This binary is visualization-only. Use ffxi_navmesh_query for --info, --query, and --world-route.\n";
        return 1;
    }

    if (args.zonePath.empty()) {
        PrintUsage(argv[0]);
        return 1;
    }

    if (!std::filesystem::exists(args.zonePath)) {
        std::cerr << "Input does not exist: " << args.zonePath << '\n';
        return 1;
    }

    dtNavMesh* navMesh = nullptr;
    if (!BuildNavMeshFromFfxiBin(args.zonePath, &navMesh)) {
        return 1;
    }

    // 3-D visualization modality.
    std::vector<Triangle> triangles = ExtractTriangles(navMesh);
    if (triangles.empty()) {
        std::cerr << "No polygons found in navmesh: " << args.zonePath << '\n';
        dtFreeNavMesh(navMesh);
        return 1;
    }

    Vector3 minBounds{}, maxBounds{};
    if (!ComputeBounds(triangles, &minBounds, &maxBounds)) {
        std::cerr << "Failed to compute bounds\n";
        dtFreeNavMesh(navMesh);
        return 1;
    }

    // RunVizMode takes ownership of navMesh; do not free it here.
    return RunVizMode(args, navMesh, args.zonePath, std::move(triangles), minBounds, maxBounds);
}

