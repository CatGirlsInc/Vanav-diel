#pragma once

#include <DetourNavMesh.h>

#include <filesystem>
#include <string>
#include <vector>

// A waypoint along a computed straight path.
struct PathWaypoint {
    float x, y, z;
    unsigned char flags; // DT_STRAIGHTPATH_* flags
};

// ─── Argument parsing ──────────────────────────────────────────────────────────

struct AppArgs {
    std::filesystem::path zonePath;
    std::filesystem::path navMeshDir = "MSETs"; // directory for zone dropdown
    std::filesystem::path dbPath = "zone_graph.db";
    bool infoOnly   = false;
    bool queryMode  = false; // CLI-only: print path to stdout, no window
    bool debugPath  = false;
    bool worldQuery = false;
    bool fileOutput = false; // print/write Lua-table friendly waypoint payloads
    bool hasFrom    = false;
    bool hasTo      = false;
    float fromPos[3] = {};
    float toPos[3]   = {};
    std::string startName;
    std::string endName;
};

bool ParseVec3(const std::string& s, float* out);
void PrintUsage(const char* exeName);
AppArgs ParseArgs(int argc, char** argv);

// ─── NavMesh loading ───────────────────────────────────────────────────────────

// Loads an FFXI .bin navmesh file into a newly allocated dtNavMesh.
// Caller is responsible for freeing the result with dtFreeNavMesh().
bool BuildNavMeshFromFfxiBin(const std::filesystem::path& path, dtNavMesh** outNavMesh);

// ─── Path computation ──────────────────────────────────────────────────────────

// Computes the straight-path waypoints between two world positions using Detour.
// Returns an empty vector on failure (unreachable, no nearby poly, etc.).
std::vector<PathWaypoint> ComputePath(
    const dtNavMesh* navMesh,
    const float* startPos,
    const float* endPos,
    bool debugPath = false);

// Prints a freshly computed path to stdout (for copy-paste convenience).
void PrintPathStdout(
    const std::vector<PathWaypoint>& path,
    const std::string& zoneName,
    const float* startPos,
    const float* endPos);

// Prints a path as a Lua table (for intra-zone payload export).
void PrintPathAsLuaTable(
    const std::vector<PathWaypoint>& path,
    int zoneId,
    const std::string& zoneName,
    const float* startPos,
    const float* endPos);

// ─── CLI query modality ────────────────────────────────────────────────────────

int RunQueryMode(
    const AppArgs& args,
    dtNavMesh* navMesh,
    const std::filesystem::path& zonePath);
