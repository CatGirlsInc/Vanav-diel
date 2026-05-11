#include "mesh_io.h"

#include <DetourNavMesh.h>

#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

// ── OBJ loader ───────────────────────────────────────────────────────────────

bool LoadObj(const std::filesystem::path& path, ObjMesh& mesh)
{
    std::ifstream f(path);
    if (!f) {
        std::cerr << "Cannot open: " << path << '\n';
        return false;
    }

    std::string line;
    while (std::getline(f, line)) {
        if (line.empty() || line[0] == '#') continue;

        std::istringstream ss(line);
        std::string token;
        ss >> token;

        if (token == "v") {
            float x, y, z;
            ss >> x >> y >> z;
            // Coordinate transformation is applied at export time (180, 0, 0 rotation).
            mesh.verts.push_back(x);
            mesh.verts.push_back(y);
            mesh.verts.push_back(z);
        } else if (token == "f") {
            // Face indices are 1-based in OBJ; support "v", "v/vt", "v/vt/vn".
            std::vector<int> idxs;
            std::string faceToken;
            while (ss >> faceToken) {
                const int idx = std::stoi(faceToken.substr(0, faceToken.find('/')));
                // Negative indices not supported (FFXI OBJs don't use them).
                idxs.push_back(idx > 0 ? idx - 1
                                       : static_cast<int>(mesh.verts.size() / 3) + idx);
            }
            // Fan-triangulate to support n-gons (OBJ files here are all
            // triangles, but be defensive).
            for (int i = 2; i < static_cast<int>(idxs.size()); ++i) {
                mesh.tris.push_back(idxs[0]);
                mesh.tris.push_back(idxs[i - 1]);
                mesh.tris.push_back(idxs[i]);
            }
        }
        // Ignore: mtllib, vt, vn, s, o, g, ...
    }

    if (mesh.verts.empty() || mesh.tris.empty()) {
        std::cerr << "No geometry in " << path << '\n';
        return false;
    }

    return true;
}

// ── MSET serialiser ──────────────────────────────────────────────────────────

bool WriteMset(const std::filesystem::path& outPath, const dtNavMesh* navMesh)
{
    std::ofstream f(outPath, std::ios::binary);
    if (!f) {
        std::cerr << "Cannot create: " << outPath << '\n';
        return false;
    }

    const dtNavMeshParams* p = navMesh->getParams();

    int numTiles = 0;
    for (int i = 0; i < navMesh->getMaxTiles(); ++i) {
        const dtMeshTile* t = navMesh->getTile(i);
        if (t && t->header && t->dataSize > 0) ++numTiles;
    }

    MsetHeader hdr{};
    hdr.magic      = kMsetMagic;
    hdr.version    = kMsetVersion;
    hdr.numTiles   = static_cast<std::uint32_t>(numTiles);
    hdr.orig[0]    = p->orig[0];
    hdr.orig[1]    = p->orig[1];
    hdr.orig[2]    = p->orig[2];
    hdr.tileWidth  = p->tileWidth;
    hdr.tileHeight = p->tileHeight;
    hdr.maxTiles   = static_cast<std::uint32_t>(p->maxTiles);
    hdr.maxPolys   = static_cast<std::uint32_t>(p->maxPolys);
    f.write(reinterpret_cast<const char*>(&hdr), sizeof(hdr));

    for (int i = 0; i < navMesh->getMaxTiles(); ++i) {
        const dtMeshTile* tile = navMesh->getTile(i);
        if (!tile || !tile->header || tile->dataSize <= 0) continue;

        MsetTileHeader th{};
        th.tileRef  = static_cast<std::uint32_t>(navMesh->getTileRef(tile));
        th.dataSize = static_cast<std::uint32_t>(tile->dataSize);
        f.write(reinterpret_cast<const char*>(&th),        sizeof(th));
        f.write(reinterpret_cast<const char*>(tile->data), tile->dataSize);
    }

    if (!f) {
        std::cerr << "Write error: " << outPath << '\n';
        return false;
    }
    return true;
}

// ── MSET deserialiser ────────────────────────────────────────────────────────

bool LoadMset(const std::filesystem::path& path, dtNavMesh** outNavMesh)
{
    std::ifstream f(path, std::ios::binary | std::ios::ate);
    if (!f) {
        std::cerr << "Cannot open: " << path << '\n';
        return false;
    }
    const std::streamsize fileSize = f.tellg();
    if (fileSize < static_cast<std::streamsize>(sizeof(MsetHeader))) {
        std::cerr << "File too small: " << path << '\n';
        return false;
    }
    f.seekg(0, std::ios::beg);

    MsetHeader hdr{};
    f.read(reinterpret_cast<char*>(&hdr), sizeof(hdr));
    if (!f) {
        std::cerr << "Read error: " << path << '\n';
        return false;
    }
    if (hdr.magic != kMsetMagic || hdr.version != kMsetVersion) {
        std::cerr << "Bad MSET header in " << path << '\n';
        return false;
    }

    dtNavMeshParams params{};
    params.orig[0]    = hdr.orig[0];
    params.orig[1]    = hdr.orig[1];
    params.orig[2]    = hdr.orig[2];
    params.tileWidth  = hdr.tileWidth;
    params.tileHeight = hdr.tileHeight;
    params.maxTiles   = static_cast<int>(hdr.maxTiles);
    params.maxPolys   = static_cast<int>(hdr.maxPolys);

    dtNavMesh* nav = dtAllocNavMesh();
    if (!nav) {
        std::cerr << "dtAllocNavMesh failed\n";
        return false;
    }
    if (dtStatusFailed(nav->init(&params))) {
        std::cerr << "dtNavMesh::init failed\n";
        dtFreeNavMesh(nav);
        return false;
    }

    for (std::uint32_t i = 0; i < hdr.numTiles; ++i) {
        MsetTileHeader th{};
        f.read(reinterpret_cast<char*>(&th), sizeof(th));
        if (!f || th.dataSize == 0) continue;

        unsigned char* data = static_cast<unsigned char*>(
            std::malloc(static_cast<std::size_t>(th.dataSize)));
        if (!data) {
            std::cerr << "OOM reading tile " << i << '\n';
            dtFreeNavMesh(nav);
            return false;
        }
        f.read(reinterpret_cast<char*>(data), th.dataSize);
        if (!f) {
            std::free(data);
            std::cerr << "Truncated tile data in " << path << '\n';
            dtFreeNavMesh(nav);
            return false;
        }
        // DT_TILE_FREE_DATA hands ownership to Detour; it calls dtFree/free.
        nav->addTile(data, static_cast<int>(th.dataSize), DT_TILE_FREE_DATA,
                     static_cast<dtTileRef>(th.tileRef), nullptr);
    }

    *outNavMesh = nav;
    return true;
}
