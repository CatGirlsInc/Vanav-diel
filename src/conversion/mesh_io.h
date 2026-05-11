#pragma once

#include <cstdint>
#include <filesystem>
#include <vector>

// ── Geometry ─────────────────────────────────────────────────────────────────

struct ObjMesh {
    std::vector<float> verts;  // packed x,y,z
    std::vector<int>   tris;   // packed i0,i1,i2 (0-based)
};

// Loads an OBJ file. Vertices and triangulated faces are populated into mesh.
// Does NOT perform winding normalization or vertex welding; call preprocessing
// steps explicitly after loading.
// Returns false on failure.
bool LoadObj(const std::filesystem::path& path, ObjMesh& mesh);

// ── MSET binary format ───────────────────────────────────────────────────────
// Little-endian binary layout consumed by FFXIPathfinding.
//
//   MsetHeader
//   For each tile:
//     MsetTileHeader
//     byte[dataSize]   – raw Detour tile blob

struct MsetHeader {
    std::uint32_t magic;      // 0x4d534554 → bytes "TESM" (LE)
    std::uint32_t version;    // 1
    std::uint32_t numTiles;
    float         orig[3];
    float         tileWidth;
    float         tileHeight;
    std::uint32_t maxTiles;
    std::uint32_t maxPolys;
};

struct MsetTileHeader {
    std::uint32_t tileRef;
    std::uint32_t dataSize;
};

static constexpr std::uint32_t kMsetMagic   = 0x4d534554u;
static constexpr std::uint32_t kMsetVersion = 1u;

class dtNavMesh;

// Writes a Detour navmesh as an MSET binary file. Returns false on failure.
bool WriteMset(const std::filesystem::path& outPath, const dtNavMesh* navMesh);

// Reads an MSET binary file and populates *outNavMesh (allocated by this
// function). Caller must free with dtFreeNavMesh(). Returns false on failure.
bool LoadMset(const std::filesystem::path& path, dtNavMesh** outNavMesh);
