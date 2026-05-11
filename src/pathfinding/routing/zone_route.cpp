#include "zone_route.h"
#include "mesh_io.h"

#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <DetourAlloc.h>

#include <cmath>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

namespace {

void PrintVec3Debug(const char* label, const float* pos) {
    std::cerr << label << ": (" << pos[0] << ", " << pos[1] << ", " << pos[2] << ")\n";
}

void PrintPathDebug(
    const float* startPos,
    const float* endPos,
    const float* snappedStart,
    const float* snappedEnd,
    dtPolyRef startRef,
    dtPolyRef endRef,
    const dtPolyRef* polys,
    int polyCount,
    const float* straightPath,
    const unsigned char* straightPathFlags,
    const dtPolyRef* straightPathRefs,
    int straightPathCount)
{
    std::cerr << "Path debug:\n";
    PrintVec3Debug("  requested start", startPos);
    PrintVec3Debug("  requested end", endPos);
    PrintVec3Debug("  snapped start", snappedStart);
    PrintVec3Debug("  snapped end", snappedEnd);
    std::cerr << std::hex;
    std::cerr << "  startRef: 0x" << startRef << "\n";
    std::cerr << "  endRef:   0x" << endRef << "\n";
    std::cerr << "  corridor polys (" << std::dec << polyCount << "):";
    for (int i = 0; i < polyCount; ++i) {
        std::cerr << " 0x" << std::hex << polys[i];
    }
    std::cerr << std::dec << "\n";
    if (polyCount > 0 && polys[polyCount - 1] != endRef) {
        std::cerr << "  partial corridor: last poly 0x" << std::hex << polys[polyCount - 1]
                  << " does not match endRef 0x" << endRef << std::dec << "\n";
    }
    std::cerr << "  straight path segments (" << straightPathCount << " points):\n";
    for (int i = 0; i < straightPathCount; ++i) {
        const float* point = &straightPath[i * 3];
        std::cerr << "    [" << i << "] (" << point[0] << ", " << point[1] << ", " << point[2] << ")";
        std::cerr << " ref=0x" << std::hex << straightPathRefs[i] << std::dec;
        std::cerr << " flags=0x" << std::hex << static_cast<unsigned int>(straightPathFlags[i]) << std::dec;
        if (i > 0) {
            const float stepYDelta = std::fabs(point[1] - straightPath[(i - 1) * 3 + 1]);
            std::cerr << "  stepY=" << stepYDelta;
        }
        std::cerr << "\n";
    }
}

constexpr int           kMaxSearchNodes      = 16384;
constexpr int           kMaxPathPolys        = 512;
constexpr int           kMaxStraightPath     = 512;

} // namespace

// ─── String utilities ──────────────────────────────────────────────────────────

static std::string EscapeLuaString(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (char ch : text) {
        if (ch == '\\' || ch == '"') {
            out.push_back('\\');
        }
        out.push_back(ch);
    }
    return out;
}

// ─── Path printing ─────────────────────────────────────────────────────────────

void PrintPathAsLuaTable(
    const std::vector<PathWaypoint>& path,
    int zoneId,
    const std::string& zoneName,
    const float* startPos,
    const float* endPos)
{
    if (path.empty()) {
        std::cout << "No path found.\n";
        return;
    }

    std::cout << std::fixed << std::setprecision(4);
    std::cout << "return {\n";
    std::cout << "\tmeta = {\n";
    std::cout << "\t\tzone_id = " << zoneId << ",\n";
    std::cout << "\t\tzone_name = \"" << EscapeLuaString(zoneName) << "\",\n";
    std::cout << "\t\tstart_coords = {x = " << startPos[0] << ", y = " << startPos[1] << ", z = " << startPos[2] << "},\n";
    std::cout << "\t\tend_coords = {x = " << endPos[0] << ", y = " << endPos[1] << ", z = " << endPos[2] << "},\n";
    std::cout << "\t},\n";
    std::cout << "\twaypoints = {\n";
    for (const PathWaypoint& wp : path) {
        std::cout << "\t\t{x = " << wp.x << ", y = " << wp.y << ", z = " << wp.z << "},\n";
    }
    std::cout << "\t},\n";
    std::cout << "}\n";
}

// ─── NavMesh loading ───────────────────────────────────────────────────────────

bool BuildNavMeshFromFfxiBin(const std::filesystem::path& path, dtNavMesh** outNavMesh) {
    return LoadMset(path, outNavMesh);
}

// ─── Path computation ──────────────────────────────────────────────────────────

std::vector<PathWaypoint> ComputePath(
    const dtNavMesh* navMesh,
    const float* startPos,
    const float* endPos,
    bool debugPath)
{
    dtNavMeshQuery* query = dtAllocNavMeshQuery();
    if (!query) {
        std::cerr << "Failed to allocate dtNavMeshQuery\n";
        return {};
    }

    dtStatus status = query->init(navMesh, kMaxSearchNodes);
    if (dtStatusFailed(status)) {
        std::cerr << "dtNavMeshQuery::init failed\n";
        dtFreeNavMeshQuery(query);
        return {};
    }

    dtQueryFilter filter;

    dtPolyRef startRef = 0;
    dtPolyRef endRef = 0;
    float snappedStart[3];
    float snappedEnd[3];
    
    // Use a slightly larger search extent than the original to handle edge cases.
    // Horizontal: 10 units (was 5), Vertical: 25 units (was 20).
    constexpr float halfExtents[3] = {10.0f, 25.0f, 10.0f};
    
    status = query->findNearestPoly(startPos, halfExtents, &filter, &startRef, snappedStart);
    if (dtStatusFailed(status) || startRef == 0) {
        std::cerr << "Could not find a polygon near the start position ("
                  << startPos[0] << ", " << startPos[1] << ", " << startPos[2] << ")\n";
        dtFreeNavMeshQuery(query);
        return {};
    }

    status = query->findNearestPoly(endPos, halfExtents, &filter, &endRef, snappedEnd);
    if (dtStatusFailed(status) || endRef == 0) {
        std::cerr << "Could not find a polygon near the end position ("
                  << endPos[0] << ", " << endPos[1] << ", " << endPos[2] << ")\n";
        dtFreeNavMeshQuery(query);
        return {};
    }
    
    dtPolyRef polys[kMaxPathPolys];
    int polyCount = 0;
    status = query->findPath(startRef, endRef, snappedStart, snappedEnd, &filter,
                             polys, &polyCount, kMaxPathPolys);
    if (dtStatusFailed(status) || polyCount == 0) {
        std::cerr << "findPath failed (status=" << std::hex << status << std::dec << ")\n";
        dtFreeNavMeshQuery(query);
        return {};
    }

    if (polys[polyCount - 1] != endRef) {
        if (debugPath) {
            std::cerr << "findPath returned a partial corridor; destination polygon is unreachable from the start polygon.\n";
            PrintVec3Debug("  requested start", startPos);
            PrintVec3Debug("  requested end", endPos);
            PrintVec3Debug("  snapped start", snappedStart);
            PrintVec3Debug("  snapped end", snappedEnd);
            std::cerr << std::hex;
            std::cerr << "  startRef: 0x" << startRef << "\n";
            std::cerr << "  endRef:   0x" << endRef << "\n";
            std::cerr << std::dec;
            std::cerr << "  corridor poly count: " << polyCount << " / " << kMaxPathPolys;
            if (polyCount == kMaxPathPolys) {
                std::cerr << " (buffer saturated)";
            }
            std::cerr << "\n";
            std::cerr << std::hex;
            std::cerr << "  last corridor poly: 0x" << polys[polyCount - 1] << "\n";
            std::cerr << std::dec;
        }
        dtFreeNavMeshQuery(query);
        return {};
    }

    float straightPath[kMaxStraightPath * 3];
    unsigned char straightPathFlags[kMaxStraightPath];
    dtPolyRef straightPathRefs[kMaxStraightPath];
    int straightPathCount = 0;

    status = query->findStraightPath(
        snappedStart, snappedEnd,
        polys, polyCount,
        straightPath, straightPathFlags, straightPathRefs,
        &straightPathCount, kMaxStraightPath);

    if (dtStatusFailed(status) || straightPathCount == 0) {
        std::cerr << "findStraightPath failed (status=" << std::hex << status << std::dec << ")\n";
        dtFreeNavMeshQuery(query);
        return {};
    }

    if (debugPath) {
        PrintPathDebug(
            startPos,
            endPos,
            snappedStart,
            snappedEnd,
            startRef,
            endRef,
            polys,
            polyCount,
            straightPath,
                straightPathFlags,
                straightPathRefs,
            straightPathCount);
    }

    std::vector<PathWaypoint> result;
    result.reserve(static_cast<std::size_t>(straightPathCount));
    for (int i = 0; i < straightPathCount; ++i) {
        result.push_back({
            straightPath[i * 3 + 0],
            straightPath[i * 3 + 1],
            straightPath[i * 3 + 2],
            straightPathFlags[i],
        });
    }

    dtFreeNavMeshQuery(query);
    return result;
}

void PrintPathStdout(
    const std::vector<PathWaypoint>& path,
    const std::string& zoneName,
    const float* startPos,
    const float* endPos)
{
    if (path.empty()) { std::cout << "No path found.\n"; return; }
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Zone:      " << zoneName << '\n';

    std::cout << "From:      " << startPos[0] << ", " << startPos[1] << ", " << startPos[2] << '\n';
    std::cout << "To:        " << endPos[0]   << ", " << endPos[1]   << ", " << endPos[2]   << '\n';
    std::cout << "Waypoints: " << path.size() << '\n';
    std::cout << "---\n";
    for (std::size_t i = 0; i < path.size(); ++i) {
        const auto& wp = path[i];
        std::string lbl;
        if (wp.flags & DT_STRAIGHTPATH_START)                   lbl = " (start)";
        else if (wp.flags & DT_STRAIGHTPATH_END)                lbl = " (end)";
        else if (wp.flags & DT_STRAIGHTPATH_OFFMESH_CONNECTION) lbl = " (offmesh)";

        std::cout << i << "  " << wp.x << ", " << wp.y << ", " << wp.z << lbl << '\n';
    }
    std::cout << "---\n";
}

// ─── Argument parsing ──────────────────────────────────────────────────────────

bool ParseVec3(const std::string& s, float* out) {
    std::string tmp = s;
    for (char& c : tmp) { if (c == ',') c = ' '; }
    std::istringstream iss(tmp);
    return static_cast<bool>(iss >> out[0] >> out[1] >> out[2]);
}

void PrintUsage(const char* exeName) {
    std::cout
        << "Usage:\n"
        << "  " << exeName << " <zone.bin> [options]\n\n"
        << "Modalities:\n"
        << "  (default)   3D viewer - visualize the nav mesh, optionally with a path\n"
        << "  --query     CLI mode  - compute path and print waypoints to stdout, no window\n\n"
        << "Options:\n"
        << "  --dir <dir>          Navmesh directory (default: MSETs)\n"
        << "  --db <path>          zone_graph.db path (default: zone_graph.db)\n"
        << "  --zone <name>        Zone name without extension\n"
        << "  --from x,y,z         Path start position (world coords)\n"
        << "  --to   x,y,z         Path end position   (world coords)\n"
        << "  --info               Print nav mesh info to stdout and exit\n"
        << "  --query              CLI path query: print waypoints, no 3D window\n\n"
        << "  --debug-path         Print snapped points, corridor poly refs, and straight-path segment deltas\n"
        << "  --file               Lua-table output for waypoint payloads (with return wrapper)\n\n"
        << "  --world-route        World route query using zone_graph.db\n"
        << "  --start-name <name>  Start NPC / anchor name\n"
        << "  --end-name <name>    End NPC / anchor name\n\n"
        << "Examples:\n"
        << "  " << exeName << " MSETs/valkurm_dunes.bin\n"
        << "  " << exeName << " MSETs/valkurm_dunes.bin --from -20,0,10 --to 50,0,-30\n"
        << "  " << exeName << " MSETs/valkurm_dunes.bin --from -20,0,10 --to 50,0,-30 --query\n"
        << "  " << exeName << " --dir MSETs --zone valkurm_dunes --info\n"
        << "  " << exeName << " --world-route --start-name \"Tahrongi Canyon Shattered Telepoint\" --end-name Horst\n";
}

AppArgs ParseArgs(int argc, char** argv) {
    AppArgs args;

    std::filesystem::path dir = "MSETs";
    std::string zone;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        if (arg == "--info") {
            args.infoOnly = true;
        } else if (arg == "--query") {
            args.queryMode = true;
        } else if (arg == "--debug-path") {
            args.debugPath = true;
        } else if (arg == "--file") {
            args.fileOutput = true;
        } else if (arg == "--world-route") {
            args.worldQuery = true;
        } else if (arg == "--dir" && i + 1 < argc) {
            dir = argv[++i];
        } else if (arg == "--db" && i + 1 < argc) {
            args.dbPath = argv[++i];
        } else if (arg == "--zone" && i + 1 < argc) {
            zone = argv[++i];
        } else if (arg == "--start-name" && i + 1 < argc) {
            args.startName = argv[++i];
        } else if (arg == "--end-name" && i + 1 < argc) {
            args.endName = argv[++i];
        } else if (arg == "--from" && i + 1 < argc) {
            if (ParseVec3(argv[++i], args.fromPos)) {
                args.hasFrom = true;
            } else {
                std::cerr << "Failed to parse --from value: " << argv[i] << '\n';
            }
        } else if (arg == "--to" && i + 1 < argc) {
            if (ParseVec3(argv[++i], args.toPos)) {
                args.hasTo = true;
            } else {
                std::cerr << "Failed to parse --to value: " << argv[i] << '\n';
            }
        } else if (arg[0] != '-') {
            args.zonePath = arg;
        }
    }

    args.navMeshDir = dir;

    if (args.zonePath.empty() && !zone.empty()) {
        args.zonePath = dir / (zone + ".bin");
    }

    return args;
}

// ─── CLI query modality ────────────────────────────────────────────────────────

int RunQueryMode(
    const AppArgs& args,
    dtNavMesh* navMesh,
    const std::filesystem::path& zonePath)
{
    if (!args.hasFrom || !args.hasTo) {
        std::cerr << "Query mode requires both --from and --to.\n";
        return 1;
    }

    float fromPos[3] = {args.fromPos[0], args.fromPos[1], args.fromPos[2]};
    float toPos[3]   = {args.toPos[0],   args.toPos[1],   args.toPos[2]};

    const std::vector<PathWaypoint> path = ComputePath(navMesh, fromPos, toPos, args.debugPath);

    if (path.empty()) {
        std::cerr << "No path found.\n";
        return 1;
    }

    if (args.fileOutput) {
        PrintPathAsLuaTable(path, 0, zonePath.stem().string(), fromPos, toPos);
        return 0;
    }
    
    std::cout << std::fixed << std::setprecision(4);
    std::cout << "Zone:  " << zonePath.filename().string() << '\n';
    std::cout << "From:  " << fromPos[0] << ", " << fromPos[1] << ", " << fromPos[2] << '\n';
    std::cout << "To:    " << toPos[0]   << ", " << toPos[1]   << ", " << toPos[2]   << '\n';

    std::cout << "Waypoints: " << path.size() << '\n';
    std::cout << "---\n";
    for (std::size_t i = 0; i < path.size(); ++i) {
        const auto& wp = path[i];
        std::string label;
        if (wp.flags & DT_STRAIGHTPATH_START)                   label = " (start)";
        else if (wp.flags & DT_STRAIGHTPATH_END)                label = " (end)";
        else if (wp.flags & DT_STRAIGHTPATH_OFFMESH_CONNECTION) label = " (offmesh)";
        std::cout << i << "  " << wp.x << ", " << wp.y << ", " << wp.z << label << '\n';
    }

    return 0;
}
