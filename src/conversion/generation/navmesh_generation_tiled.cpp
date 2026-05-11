#include "navmesh_generation_tiled.h"

#include <DetourAlloc.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <Recast.h>

#include <cmath>
#include <iostream>
#include <unordered_map>
#include <vector>

namespace {

using SilentContext = rcContext;

struct RecastBuild {
    rcHeightfield*        hf = nullptr;
    rcCompactHeightfield* chf = nullptr;
    rcContourSet*         cset = nullptr;
    rcPolyMesh*           pmesh = nullptr;
    rcPolyMeshDetail*     dmesh = nullptr;

    ~RecastBuild()
    {
        rcFreeHeightField(hf);
        rcFreeCompactHeightfield(chf);
        rcFreeContourSet(cset);
        rcFreePolyMesh(pmesh);
        rcFreePolyMeshDetail(dmesh);
    }
};

struct ConnectivityStats {
    int totalPolys = 0;
    int largestComponentPolys = 0;
    int components = 0;
};

int NextPow2(int v)
{
    int x = 1;
    while (x < v && x > 0) x <<= 1;
    return x > 0 ? x : (1 << 30);
}

int ILog2(int v)
{
    int r = 0;
    while (v > 1) {
        v >>= 1;
        ++r;
    }
    return r;
}

ConnectivityStats ComputeConnectivityStats(const dtNavMesh* navMesh)
{
    ConnectivityStats stats{};
    std::vector<dtPolyRef> refs;
    refs.reserve(32768);

    const int maxTiles = navMesh->getMaxTiles();
    for (int ti = 0; ti < maxTiles; ++ti) {
        const dtMeshTile* tile = navMesh->getTile(ti);
        if (!tile || !tile->header) continue;

        for (int pi = 0; pi < tile->header->polyCount; ++pi) {
            const dtPoly* poly = &tile->polys[pi];
            if (poly->flags == 0 || poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION) {
                continue;
            }
            refs.push_back(navMesh->getPolyRefBase(tile) | static_cast<dtPolyRef>(pi));
        }
    }

    stats.totalPolys = static_cast<int>(refs.size());
    if (refs.empty()) return stats;

    std::unordered_map<dtPolyRef, int> refToIdx;
    refToIdx.reserve(refs.size() * 2u);
    for (int i = 0; i < static_cast<int>(refs.size()); ++i) {
        refToIdx[refs[i]] = i;
    }

    std::vector<char> visited(refs.size(), 0);
    std::vector<int> queue;
    queue.reserve(refs.size());

    for (int i = 0; i < static_cast<int>(refs.size()); ++i) {
        if (visited[i]) continue;
        ++stats.components;
        int compSize = 0;

        queue.clear();
        queue.push_back(i);
        visited[i] = 1;

        for (std::size_t q = 0; q < queue.size(); ++q) {
            const int curIdx = queue[q];
            ++compSize;

            dtPolyRef curRef = refs[curIdx];
            const dtMeshTile* tile = nullptr;
            const dtPoly* poly = nullptr;
            if (dtStatusFailed(navMesh->getTileAndPolyByRef(curRef, &tile, &poly))) continue;

            for (unsigned int link = poly->firstLink; link != DT_NULL_LINK; link = tile->links[link].next) {
                const dtPolyRef nbRef = tile->links[link].ref;
                if (!nbRef) continue;

                auto it = refToIdx.find(nbRef);
                if (it == refToIdx.end()) continue;
                if (visited[it->second]) continue;
                visited[it->second] = 1;
                queue.push_back(it->second);
            }
        }

        if (compSize > stats.largestComponentPolys) {
            stats.largestComponentPolys = compSize;
        }
    }

    return stats;
}

} // namespace

bool GenerateTiledNavMesh(const ObjMesh&     mesh,
                          const BuildConfig& cfg,
                          float              cellSize,
                          int                navMinComponentPolys,
                          dtNavMesh**        outNavMesh)
{
    *outNavMesh = nullptr;
    SilentContext ctx;

    const float* verts = mesh.verts.data();
    const int nVerts = static_cast<int>(mesh.verts.size()) / 3;
    const int* tris = mesh.tris.data();
    const int nTris = static_cast<int>(mesh.tris.size()) / 3;

    float bmin[3], bmax[3];
    const float pad = cfg.agentRadius * 3.0f;
    ComputeMeshBounds(mesh, pad, bmin, bmax);

    rcConfig base{};
    base.cs = cellSize;
    base.ch = cfg.ch;
    base.walkableSlopeAngle = cfg.slopeAngle;
    base.walkableHeight = static_cast<int>(std::ceil(cfg.agentHeight / cfg.ch));
    base.walkableClimb = static_cast<int>(std::ceil(cfg.agentClimb / cfg.ch));
    base.walkableRadius = static_cast<int>(std::ceil(cfg.agentRadius / base.cs));
    base.maxEdgeLen = static_cast<int>(cfg.edgeMaxLen / base.cs);
    base.maxSimplificationError = cfg.edgeMaxError;
    base.minRegionArea = cfg.regionMinSize;
    base.mergeRegionArea = cfg.regionMergeSize;
    base.maxVertsPerPoly = cfg.vertsPerPoly;
    base.detailSampleDist = base.cs * cfg.detailSampleDist;
    base.detailSampleMaxError = cfg.ch * cfg.detailSampleMaxErr;

    const int tileSize = std::max(32, cfg.tileSizeCells);
    int gw = 0;
    int gh = 0;
    rcCalcGridSize(bmin, bmax, base.cs, &gw, &gh);
    const int tw = (gw + tileSize - 1) / tileSize;
    const int th = (gh + tileSize - 1) / tileSize;
    const int expectedTiles = std::max(1, tw * th);

    dtNavMeshParams params{};
    rcVcopy(params.orig, bmin);
    params.tileWidth = tileSize * base.cs;
    params.tileHeight = tileSize * base.cs;
    params.maxTiles = expectedTiles;
    params.maxPolys = std::max(64, cfg.maxPolysPerTile);

    // Detour poly refs have a fixed 22-bit budget split between tile and poly
    // indices. Smaller tile sizes increase tile count, so clamp maxPolys to
    // keep tileBits+polyBits <= 22 and avoid init failures.
    const int tileBits = ILog2(NextPow2(params.maxTiles));
    const int polyBits = ILog2(NextPow2(params.maxPolys));
    static constexpr int kRefBudgetBits = 22;
    if (tileBits + polyBits > kRefBudgetBits) {
        const int allowedPolyBits = std::max(1, kRefBudgetBits - tileBits);
        const int adjustedMaxPolys = 1 << allowedPolyBits;
        std::cerr << "  [tiled] adjusting max-polys-per-tile from "
                  << params.maxPolys << " to " << adjustedMaxPolys
                  << " to satisfy Detour ref bit budget"
                  << " (tileBits=" << tileBits
                  << ", polyBits=" << polyBits << ")\n";
        params.maxPolys = adjustedMaxPolys;
    }

    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        std::cerr << "dtAllocNavMesh failed\n";
        return false;
    }
    if (dtStatusFailed(navMesh->init(&params))) {
        std::cerr << "dtNavMesh::init failed for tiled mesh\n";
        dtFreeNavMesh(navMesh);
        return false;
    }

    std::cerr << "  [tiled] attempting tiled generation @ cs=" << cellSize
              << " with " << tw << "x" << th << " tiles (tileSize="
              << tileSize << " cells, maxPolysPerTile="
              << params.maxPolys << ")\n";

    int builtTiles = 0;

    for (int ty = 0; ty < th; ++ty) {
        for (int tx = 0; tx < tw; ++tx) {
            rcConfig rc = base;
            rc.tileSize = tileSize;
            rc.borderSize = rc.walkableRadius + 3;
            rc.width = rc.tileSize + rc.borderSize * 2;
            rc.height = rc.tileSize + rc.borderSize * 2;

            rc.bmin[0] = bmin[0] + (tx * rc.tileSize - rc.borderSize) * rc.cs;
            rc.bmin[1] = bmin[1];
            rc.bmin[2] = bmin[2] + (ty * rc.tileSize - rc.borderSize) * rc.cs;
            rc.bmax[0] = bmin[0] + ((tx + 1) * rc.tileSize + rc.borderSize) * rc.cs;
            rc.bmax[1] = bmax[1];
            rc.bmax[2] = bmin[2] + ((ty + 1) * rc.tileSize + rc.borderSize) * rc.cs;

            RecastBuild b;
            b.hf = rcAllocHeightfield();
            if (!b.hf || !rcCreateHeightfield(&ctx, *b.hf, rc.width, rc.height,
                                              rc.bmin, rc.bmax, rc.cs, rc.ch)) {
                std::cerr << "rcCreateHeightfield failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }

            std::vector<unsigned char> triAreas(static_cast<std::size_t>(nTris), RC_NULL_AREA);
            rcMarkWalkableTriangles(&ctx, rc.walkableSlopeAngle,
                                    verts, nVerts, tris, nTris, triAreas.data());
            if (!rcRasterizeTriangles(&ctx, verts, nVerts, tris, triAreas.data(),
                                      nTris, *b.hf, rc.walkableClimb)) {
                std::cerr << "rcRasterizeTriangles failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }

            rcFilterLowHangingWalkableObstacles(&ctx, rc.walkableClimb, *b.hf);
            rcFilterLedgeSpans(&ctx, rc.walkableHeight, rc.walkableClimb, *b.hf);
            rcFilterWalkableLowHeightSpans(&ctx, rc.walkableHeight, *b.hf);

            b.chf = rcAllocCompactHeightfield();
            if (!b.chf || !rcBuildCompactHeightfield(&ctx, rc.walkableHeight,
                                                     rc.walkableClimb, *b.hf, *b.chf)) {
                std::cerr << "rcBuildCompactHeightfield failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }
            if (!rcErodeWalkableArea(&ctx, rc.walkableRadius, *b.chf)) {
                std::cerr << "rcErodeWalkableArea failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }
            if (!rcBuildDistanceField(&ctx, *b.chf)) {
                std::cerr << "rcBuildDistanceField failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }
            if (!rcBuildRegions(&ctx, *b.chf, rc.borderSize, rc.minRegionArea, rc.mergeRegionArea)) {
                std::cerr << "rcBuildRegions failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }

            b.cset = rcAllocContourSet();
            if (!b.cset || !rcBuildContours(&ctx, *b.chf, rc.maxSimplificationError,
                                            rc.maxEdgeLen, *b.cset)) {
                std::cerr << "rcBuildContours failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }
            if (b.cset->nconts == 0) {
                continue;
            }

            b.pmesh = rcAllocPolyMesh();
            if (!b.pmesh || !rcBuildPolyMesh(&ctx, *b.cset, rc.maxVertsPerPoly, *b.pmesh)) {
                std::cerr << "rcBuildPolyMesh failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }

            if (navMinComponentPolys > 0) {
                PruneSmallNavPolyComponents(*b.pmesh, navMinComponentPolys);
            }

            b.dmesh = rcAllocPolyMeshDetail();
            if (!b.dmesh || !rcBuildPolyMeshDetail(&ctx, *b.pmesh, *b.chf,
                                                   rc.detailSampleDist,
                                                   rc.detailSampleMaxError, *b.dmesh)) {
                std::cerr << "rcBuildPolyMeshDetail failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }

            for (int i = 0; i < b.pmesh->npolys; ++i) {
                if (b.pmesh->areas[i] != RC_NULL_AREA) {
                    b.pmesh->areas[i] = 1;
                    b.pmesh->flags[i] = 1;
                } else {
                    b.pmesh->flags[i] = 0;
                }
            }

            dtNavMeshCreateParams cp{};
            cp.verts            = b.pmesh->verts;
            cp.vertCount        = b.pmesh->nverts;
            cp.polys            = b.pmesh->polys;
            cp.polyAreas        = b.pmesh->areas;
            cp.polyFlags        = b.pmesh->flags;
            cp.polyCount        = b.pmesh->npolys;
            cp.nvp              = b.pmesh->nvp;
            cp.detailMeshes     = b.dmesh->meshes;
            cp.detailVerts      = b.dmesh->verts;
            cp.detailVertsCount = b.dmesh->nverts;
            cp.detailTris       = b.dmesh->tris;
            cp.detailTriCount   = b.dmesh->ntris;
            cp.walkableHeight   = cfg.agentHeight;
            cp.walkableRadius   = cfg.agentRadius;
            cp.walkableClimb    = cfg.agentClimb;
            cp.tileX            = tx;
            cp.tileY            = ty;
            cp.tileLayer        = 0;
            cp.cs               = rc.cs;
            cp.ch               = rc.ch;
            cp.buildBvTree      = true;
            rcVcopy(cp.bmin, b.pmesh->bmin);
            rcVcopy(cp.bmax, b.pmesh->bmax);

            unsigned char* tileData = nullptr;
            int tileDataSize = 0;
            if (!dtCreateNavMeshData(&cp, &tileData, &tileDataSize)) {
                std::cerr << "dtCreateNavMeshData failed (tile " << tx << "," << ty << ")\n";
                dtFreeNavMesh(navMesh);
                return false;
            }

            const dtStatus addStatus = navMesh->addTile(tileData, tileDataSize,
                                                        DT_TILE_FREE_DATA, 0, nullptr);
            if (dtStatusFailed(addStatus)) {
                std::cerr << "addTile failed (tile " << tx << "," << ty << ")\n";
                dtFree(tileData);
                dtFreeNavMesh(navMesh);
                return false;
            }

            ++builtTiles;
        }
    }

    if (builtTiles == 0) {
        std::cerr << "tiled generation produced no tiles\n";
        dtFreeNavMesh(navMesh);
        return false;
    }

    const ConnectivityStats conn = ComputeConnectivityStats(navMesh);
    const float largestRatio = (conn.totalPolys > 0)
        ? static_cast<float>(conn.largestComponentPolys) / static_cast<float>(conn.totalPolys)
        : 0.0f;
    std::cerr << "  [tiled] built " << builtTiles << " populated tiles\n";
    std::cerr << "  [tiled] connectivity: largest-component="
              << conn.largestComponentPolys << "/" << conn.totalPolys
              << " polys across " << conn.components << " components"
              << " (ratio=" << largestRatio << ")\n";

    static constexpr float kMinLargestComponentRatio = 0.60f;
    if (largestRatio < kMinLargestComponentRatio) {
        std::cerr << "  [tiled] rejected: connectivity ratio below threshold "
                  << kMinLargestComponentRatio << "\n";
        dtFreeNavMesh(navMesh);
        return false;
    }

    *outNavMesh = navMesh;
    return true;
}
