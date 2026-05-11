#include "routing/zone_route.h"
#include "routing/world_route.h"

#include <DetourAlloc.h>
#include <DetourNavMesh.h>

#include <algorithm>
#include <filesystem>
#include <iostream>
#include <optional>
#include <string>
#include <vector>

namespace {

struct MeshInfo {
    std::size_t triangleCount = 0;
    bool hasBounds = false;
    float min[3] = {0.0f, 0.0f, 0.0f};
    float max[3] = {0.0f, 0.0f, 0.0f};
};

MeshInfo ComputeMeshInfo(const dtNavMesh* navMesh) {
    MeshInfo info;
    const int maxTiles = navMesh->getMaxTiles();

    for (int tileIndex = 0; tileIndex < maxTiles; ++tileIndex) {
        const dtMeshTile* tile = navMesh->getTile(tileIndex);
        if (!tile || !tile->header) {
            continue;
        }

        for (int polyIndex = 0; polyIndex < tile->header->polyCount; ++polyIndex) {
            const dtPoly* poly = &tile->polys[polyIndex];
            if (poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION || poly->vertCount < 3) {
                continue;
            }

            info.triangleCount += static_cast<std::size_t>(poly->vertCount - 2);

            for (unsigned int i = 0; i < poly->vertCount; ++i) {
                const unsigned int vi = poly->verts[i];
                const float* v = &tile->verts[vi * 3];
                if (!info.hasBounds) {
                    info.min[0] = info.max[0] = v[0];
                    info.min[1] = info.max[1] = v[1];
                    info.min[2] = info.max[2] = v[2];
                    info.hasBounds = true;
                    continue;
                }
                info.min[0] = std::min(info.min[0], v[0]);
                info.min[1] = std::min(info.min[1], v[1]);
                info.min[2] = std::min(info.min[2], v[2]);
                info.max[0] = std::max(info.max[0], v[0]);
                info.max[1] = std::max(info.max[1], v[1]);
                info.max[2] = std::max(info.max[2], v[2]);
            }
        }
    }

    return info;
}

void PrintQueryUsage(const char* exeName) {
    std::cout
        << "Usage:\n"
        << "  " << exeName << " <zone.bin> [--query] --from x,y,z --to x,y,z\n"
        << "  " << exeName << " <zone.bin> --info\n"
        << "  " << exeName << " --start-name <name> --end-name <name> [--db zone_graph.db] [--file]\n"
        << "  " << exeName << " --world-route --start-name <name> --end-name <name> [--db zone_graph.db] [--file]\n\n"
        << "Options:\n"
        << "  --dir <dir>          Navmesh directory (default: MSETs)\n"
        << "  --db <path>          zone_graph.db path (default: zone_graph.db)\n"
        << "  --zone <name>        Zone name without extension\n"
        << "  --from x,y,z         Path start position\n"
        << "  --to   x,y,z         Path end position\n"
        << "  --query              Force path query mode\n"
        << "  --info               Print nav mesh info\n"
        << "  --world-route        World route query (optional when start/end names are provided)\n"
        << "  --start-name <name>  Start NPC / anchor name\n"
        << "  --end-name <name>    End NPC / anchor name\n"
        << "  --file               Lua output for intra-zone path payloads (with return wrapper)\n";
}

} // namespace

int main(int argc, char** argv) {
    const AppArgs args = ParseArgs(argc, argv);
    const bool worldMode = args.worldQuery || !args.startName.empty() || !args.endName.empty();

    if (worldMode) {
        if (args.startName.empty() || args.endName.empty()) {
            std::cerr << "World route mode requires --start-name and --end-name.\n";
            return 1;
        }

        WorldGraph graph;
        std::string loadError;
        if (!LoadWorldGraph(args.dbPath, &graph, &loadError)) {
            std::cerr << "Failed to load world graph from " << args.dbPath << ": " << loadError << '\n';
            return 1;
        }

        std::vector<std::string> warnings;
        const std::optional<WorldTarget> start = ResolveWorldTarget(graph, args.startName, &warnings);
        const std::optional<WorldTarget> end = ResolveWorldTarget(graph, args.endName, &warnings);
        for (const std::string& warning : warnings) {
            std::cerr << "warning: " << warning << '\n';
        }

        if (!start || !end) {
            std::cerr << "Could not resolve one or both world targets.\n";
            return 1;
        }

        const std::optional<WorldRoute> route = FindWorldRoute(graph, *start, *end, &warnings);
        for (const std::string& warning : warnings) {
            std::cerr << "warning: " << warning << '\n';
        }

        if (!route) {
            std::cerr << "No world route found.\n";
            return 1;
        }

        PrintWorldRoute(*route, graph, args.navMeshDir, args.fileOutput, &warnings);
        for (const std::string& warning : warnings) {
            std::cerr << "warning: " << warning << '\n';
        }
        return 0;
    }

    if (args.zonePath.empty()) {
        PrintQueryUsage(argv[0]);
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

    if (args.infoOnly) {
        const MeshInfo info = ComputeMeshInfo(navMesh);
        std::cout << "Loaded zone: " << args.zonePath << '\n';
        std::cout << "Triangles: " << info.triangleCount << '\n';
        if (info.hasBounds) {
            std::cout << "Bounds min: (" << info.min[0] << ", " << info.min[1] << ", " << info.min[2] << ")\n";
            std::cout << "Bounds max: (" << info.max[0] << ", " << info.max[1] << ", " << info.max[2] << ")\n";
        } else {
            std::cout << "Bounds: (empty navmesh)\n";
        }
        dtFreeNavMesh(navMesh);
        return 0;
    }

    if (!args.queryMode && !(args.hasFrom && args.hasTo)) {
        PrintQueryUsage(argv[0]);
        dtFreeNavMesh(navMesh);
        return 1;
    }

    const int ret = RunQueryMode(args, navMesh, args.zonePath);
    dtFreeNavMesh(navMesh);
    return ret;
}
