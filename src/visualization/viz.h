#pragma once

#include <raylib.h>

#include <DetourNavMesh.h>

#include "pathfinding/routing/zone_route.h"

#include <filesystem>
#include <string>
#include <vector>

// A triangle on the nav mesh surface, used for rendering and raycasting.
struct Triangle {
    Vector3 a;
    Vector3 b;
    Vector3 c;
    int componentId = -1;
};

// ─── Zone browser types ────────────────────────────────────────────────────────

struct ZoneEntry {
    std::string name;           // e.g. "western_altepa_desert"
    std::filesystem::path path;
};

struct ZoneTransition {
    Vector3 from;
    Vector3 to;
    std::string toZoneName;
};

// ─── NavMesh geometry helpers ──────────────────────────────────────────────────

// Extracts all walkable triangles from a loaded dtNavMesh for rendering/raycasting.
std::vector<Triangle> ExtractTriangles(const dtNavMesh* navMesh);

// Computes the axis-aligned bounding box of a triangle set. Returns false if empty.
bool ComputeBounds(const std::vector<Triangle>& triangles, Vector3* outMin, Vector3* outMax);

// ─── Zone browser helpers ──────────────────────────────────────────────────────

// Returns all .bin files under dir, sorted by name.
std::vector<ZoneEntry> ScanZoneDir(const std::filesystem::path& dir);

// Loads zone transition records from a _connections.json file adjacent to zoneBinPath.
std::vector<ZoneTransition> LoadZoneTransitions(const std::filesystem::path& zoneBinPath);

// ─── 3D visualization modality ─────────────────────────────────────────────────
//
// RunVizMode takes ownership of initialNavMesh and frees it (or its replacement)
// on exit.  The caller must NOT free navMesh after calling this function.

int RunVizMode(
    const AppArgs& args,
    dtNavMesh* initialNavMesh,
    const std::filesystem::path& zonePath,
    std::vector<Triangle> triangles,
    Vector3 minBounds,
    Vector3 maxBounds);
