#include "navmesh_generation_monolithic.h"

#include <DetourAlloc.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshBuilder.h>
#include <Recast.h>

#include <cmath>
#include <iostream>
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

} // namespace

bool GenerateMonolithicNavMesh(const ObjMesh&     mesh,
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

    rcConfig rc{};
    rc.cs = cellSize;
    rc.ch = cfg.ch;
    rc.walkableSlopeAngle = cfg.slopeAngle;
    rc.walkableHeight = static_cast<int>(std::ceil(cfg.agentHeight / cfg.ch));
    rc.walkableClimb = static_cast<int>(std::ceil(cfg.agentClimb / cfg.ch));
    rc.walkableRadius = static_cast<int>(std::ceil(cfg.agentRadius / rc.cs));
    rc.maxEdgeLen = static_cast<int>(cfg.edgeMaxLen / rc.cs);
    rc.maxSimplificationError = cfg.edgeMaxError;
    rc.minRegionArea = cfg.regionMinSize;
    rc.mergeRegionArea = cfg.regionMergeSize;
    rc.maxVertsPerPoly = cfg.vertsPerPoly;
    rc.detailSampleDist = rc.cs * cfg.detailSampleDist;
    rc.detailSampleMaxError = cfg.ch * cfg.detailSampleMaxErr;
    rcVcopy(rc.bmin, bmin);
    rcVcopy(rc.bmax, bmax);
    rcCalcGridSize(rc.bmin, rc.bmax, rc.cs, &rc.width, &rc.height);

    std::cerr << "  [diag] input: " << nTris << " tris, " << nVerts
              << " verts | grid: " << rc.width << " x " << rc.height
              << " cells @ cs=" << rc.cs << "\n";

    RecastBuild b;

    b.hf = rcAllocHeightfield();
    if (!b.hf || !rcCreateHeightfield(&ctx, *b.hf, rc.width, rc.height,
                                      rc.bmin, rc.bmax, rc.cs, rc.ch)) {
        std::cerr << "rcCreateHeightfield failed\n";
        return false;
    }

    std::vector<unsigned char> triAreas(static_cast<std::size_t>(nTris), RC_NULL_AREA);
    rcMarkWalkableTriangles(&ctx, rc.walkableSlopeAngle,
                            verts, nVerts, tris, nTris, triAreas.data());
    if (!rcRasterizeTriangles(&ctx, verts, nVerts, tris, triAreas.data(),
                              nTris, *b.hf, rc.walkableClimb)) {
        std::cerr << "rcRasterizeTriangles failed\n";
        return false;
    }

    rcFilterLowHangingWalkableObstacles(&ctx, rc.walkableClimb, *b.hf);
    rcFilterLedgeSpans(&ctx, rc.walkableHeight, rc.walkableClimb, *b.hf);
    rcFilterWalkableLowHeightSpans(&ctx, rc.walkableHeight, *b.hf);

    b.chf = rcAllocCompactHeightfield();
    if (!b.chf || !rcBuildCompactHeightfield(&ctx, rc.walkableHeight,
                                             rc.walkableClimb, *b.hf, *b.chf)) {
        std::cerr << "rcBuildCompactHeightfield failed\n";
        return false;
    }
    if (!rcErodeWalkableArea(&ctx, rc.walkableRadius, *b.chf)) {
        std::cerr << "rcErodeWalkableArea failed\n";
        return false;
    }
    if (!rcBuildDistanceField(&ctx, *b.chf)) {
        std::cerr << "rcBuildDistanceField failed\n";
        return false;
    }
    if (!rcBuildRegions(&ctx, *b.chf, 0, rc.minRegionArea, rc.mergeRegionArea)) {
        std::cerr << "rcBuildRegions failed\n";
        return false;
    }

    b.cset = rcAllocContourSet();
    if (!b.cset || !rcBuildContours(&ctx, *b.chf, rc.maxSimplificationError,
                                    rc.maxEdgeLen, *b.cset)) {
        std::cerr << "rcBuildContours failed\n";
        return false;
    }
    if (b.cset->nconts == 0) {
        std::cerr << "No contours generated\n";
        return false;
    }
    std::cerr << "  [diag] contours: " << b.cset->nconts << "\n";

    int contourTotalVerts = 0;
    int contourMaxVerts = 0;
    for (int ci = 0; ci < b.cset->nconts; ++ci) {
        const int nv = b.cset->conts[ci].nverts;
        if (nv < 3) continue;
        contourTotalVerts += nv;
        contourMaxVerts = std::max(contourMaxVerts, nv);
    }
    std::cerr << "  [diag] contour verts: total=" << contourTotalVerts
              << ", max-per-contour=" << contourMaxVerts
              << ", rcBuildPolyMesh limit(total)<65534\n";
    if (contourTotalVerts >= 0xfffe) {
        std::cerr << "  [diag] expected rcBuildPolyMesh failure: Too many vertices ("
                  << contourTotalVerts << ")\n";
    }

    b.pmesh = rcAllocPolyMesh();
    if (!b.pmesh || !rcBuildPolyMesh(&ctx, *b.cset, rc.maxVertsPerPoly, *b.pmesh)) {
        std::cerr << "rcBuildPolyMesh failed\n";
        return false;
    }

    if (navMinComponentPolys > 0) {
        const NavComponentPruneStats cps =
            PruneSmallNavPolyComponents(*b.pmesh, navMinComponentPolys);
        if (cps.totalComponents > 0) {
            std::cerr << "  [nav component prune] kept " << cps.keptPolys
                      << " polys across " << cps.keptComponents << "/"
                      << cps.totalComponents << " components"
                      << " (min=" << navMinComponentPolys
                      << ", pruned=" << cps.prunedPolys << " polys in "
                      << cps.prunedComponents << " components)\n";
        }
    }

    b.dmesh = rcAllocPolyMeshDetail();
    if (!b.dmesh || !rcBuildPolyMeshDetail(&ctx, *b.pmesh, *b.chf,
                                           rc.detailSampleDist,
                                           rc.detailSampleMaxError, *b.dmesh)) {
        std::cerr << "rcBuildPolyMeshDetail failed\n";
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

    if (rc.maxVertsPerPoly > DT_VERTS_PER_POLYGON) {
        std::cerr << "maxVertsPerPoly exceeds DT_VERTS_PER_POLYGON\n";
        return false;
    }

    dtNavMeshCreateParams params{};
    params.verts            = b.pmesh->verts;
    params.vertCount        = b.pmesh->nverts;
    params.polys            = b.pmesh->polys;
    params.polyAreas        = b.pmesh->areas;
    params.polyFlags        = b.pmesh->flags;
    params.polyCount        = b.pmesh->npolys;
    params.nvp              = b.pmesh->nvp;
    params.detailMeshes     = b.dmesh->meshes;
    params.detailVerts      = b.dmesh->verts;
    params.detailVertsCount = b.dmesh->nverts;
    params.detailTris       = b.dmesh->tris;
    params.detailTriCount   = b.dmesh->ntris;
    params.walkableHeight   = cfg.agentHeight;
    params.walkableRadius   = cfg.agentRadius;
    params.walkableClimb    = cfg.agentClimb;
    rcVcopy(params.bmin, b.pmesh->bmin);
    rcVcopy(params.bmax, b.pmesh->bmax);
    params.cs               = rc.cs;
    params.ch               = rc.ch;
    params.buildBvTree      = true;

    unsigned char* navData = nullptr;
    int navDataSize = 0;
    if (!dtCreateNavMeshData(&params, &navData, &navDataSize)) {
        std::cerr << "dtCreateNavMeshData failed\n";
        return false;
    }

    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        dtFree(navData);
        std::cerr << "dtAllocNavMesh failed\n";
        return false;
    }
    const dtStatus st = navMesh->init(navData, navDataSize, DT_TILE_FREE_DATA);
    if (dtStatusFailed(st)) {
        std::cerr << "dtNavMesh::init failed\n";
        dtFreeNavMesh(navMesh);
        return false;
    }

    *outNavMesh = navMesh;
    return true;
}
