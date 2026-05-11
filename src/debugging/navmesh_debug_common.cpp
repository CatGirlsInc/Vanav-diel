#include "debugging/navmesh_debug_common.h"

#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

#include <algorithm>
#include <cmath>
#include <limits>
#include <queue>
#include <vector>

namespace debugnav {
namespace {

constexpr float kDefaultPointExtents[3] = {2.0f, 4.0f, 2.0f};
constexpr float kProbePointExtents[3] = {4.0f, 6.0f, 4.0f};

bool IsWalkablePoly(const dtPoly* poly) {
    return poly && poly->flags != 0 && poly->getType() != DT_POLYTYPE_OFFMESH_CONNECTION && poly->vertCount >= 3;
}

bool GetPolyBoundsAndCenter(
    const dtMeshTile* tile,
    const dtPoly* poly,
    float outMin[3],
    float outMax[3],
    float outCenter[3])
{
    if (!tile || !poly || poly->vertCount == 0) {
        return false;
    }

    outMin[0] = outMax[0] = tile->verts[poly->verts[0] * 3 + 0];
    outMin[1] = outMax[1] = tile->verts[poly->verts[0] * 3 + 1];
    outMin[2] = outMax[2] = tile->verts[poly->verts[0] * 3 + 2];

    float sum[3] = {0.0f, 0.0f, 0.0f};

    for (unsigned int i = 0; i < poly->vertCount; ++i) {
        const float* v = &tile->verts[poly->verts[i] * 3];
        outMin[0] = std::min(outMin[0], v[0]);
        outMin[1] = std::min(outMin[1], v[1]);
        outMin[2] = std::min(outMin[2], v[2]);
        outMax[0] = std::max(outMax[0], v[0]);
        outMax[1] = std::max(outMax[1], v[1]);
        outMax[2] = std::max(outMax[2], v[2]);
        sum[0] += v[0];
        sum[1] += v[1];
        sum[2] += v[2];
    }

    const float invCount = 1.0f / static_cast<float>(poly->vertCount);
    outCenter[0] = sum[0] * invCount;
    outCenter[1] = sum[1] * invCount;
    outCenter[2] = sum[2] * invCount;
    return true;
}

void GrowBounds(float dstMin[3], float dstMax[3], const float srcMin[3], const float srcMax[3]) {
    dstMin[0] = std::min(dstMin[0], srcMin[0]);
    dstMin[1] = std::min(dstMin[1], srcMin[1]);
    dstMin[2] = std::min(dstMin[2], srcMin[2]);
    dstMax[0] = std::max(dstMax[0], srcMax[0]);
    dstMax[1] = std::max(dstMax[1], srcMax[1]);
    dstMax[2] = std::max(dstMax[2], srcMax[2]);
}

float Distance3(const float* a, const float* b) {
    const float dx = a[0] - b[0];
    const float dy = a[1] - b[1];
    const float dz = a[2] - b[2];
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

void AppendUnique(std::vector<int>& values, int v) {
    if (values.empty() || values.back() != v) {
        values.push_back(v);
    }
}

} // namespace

ConnectivityIndex BuildConnectivityIndex(const dtNavMesh* navMesh) {
    ConnectivityIndex index;
    if (!navMesh) {
        return index;
    }

    std::vector<dtPolyRef> walkableRefs;
    const int maxTiles = navMesh->getMaxTiles();

    for (int tileIndex = 0; tileIndex < maxTiles; ++tileIndex) {
        const dtMeshTile* tile = navMesh->getTile(tileIndex);
        if (!tile || !tile->header) {
            continue;
        }

        const dtPolyRef baseRef = navMesh->getPolyRefBase(tile);
        for (int polyIndex = 0; polyIndex < tile->header->polyCount; ++polyIndex) {
            const dtPoly* poly = &tile->polys[polyIndex];
            if (!IsWalkablePoly(poly)) {
                continue;
            }
            walkableRefs.push_back(baseRef | static_cast<dtPolyRef>(polyIndex));
        }
    }

    std::sort(walkableRefs.begin(), walkableRefs.end());
    index.walkablePolyCount = static_cast<int>(walkableRefs.size());

    std::queue<dtPolyRef> q;
    for (dtPolyRef seedRef : walkableRefs) {
        if (index.polyToComponent.find(seedRef) != index.polyToComponent.end()) {
            continue;
        }

        ComponentInfo info;
        info.componentId = static_cast<int>(index.components.size());
        info.minBounds[0] = info.minBounds[1] = info.minBounds[2] = std::numeric_limits<float>::max();
        info.maxBounds[0] = info.maxBounds[1] = info.maxBounds[2] = -std::numeric_limits<float>::max();

        float centroidAccum[3] = {0.0f, 0.0f, 0.0f};

        index.polyToComponent[seedRef] = info.componentId;
        q.push(seedRef);

        while (!q.empty()) {
            const dtPolyRef ref = q.front();
            q.pop();

            const dtMeshTile* tile = nullptr;
            const dtPoly* poly = nullptr;
            if (dtStatusFailed(navMesh->getTileAndPolyByRef(ref, &tile, &poly)) || !IsWalkablePoly(poly)) {
                continue;
            }

            ++info.polyCount;

            float polyMin[3], polyMax[3], polyCenter[3];
            if (GetPolyBoundsAndCenter(tile, poly, polyMin, polyMax, polyCenter)) {
                GrowBounds(info.minBounds, info.maxBounds, polyMin, polyMax);
                centroidAccum[0] += polyCenter[0];
                centroidAccum[1] += polyCenter[1];
                centroidAccum[2] += polyCenter[2];
            }

            for (unsigned int linkIndex = poly->firstLink; linkIndex != DT_NULL_LINK; linkIndex = tile->links[linkIndex].next) {
                const dtPolyRef neiRef = tile->links[linkIndex].ref;
                if (!neiRef) {
                    continue;
                }
                if (index.polyToComponent.find(neiRef) != index.polyToComponent.end()) {
                    continue;
                }

                const dtMeshTile* neiTile = nullptr;
                const dtPoly* neiPoly = nullptr;
                if (dtStatusFailed(navMesh->getTileAndPolyByRef(neiRef, &neiTile, &neiPoly)) || !IsWalkablePoly(neiPoly)) {
                    continue;
                }

                index.polyToComponent[neiRef] = info.componentId;
                q.push(neiRef);
            }
        }

        if (info.polyCount > 0) {
            const float inv = 1.0f / static_cast<float>(info.polyCount);
            info.centroid[0] = centroidAccum[0] * inv;
            info.centroid[1] = centroidAccum[1] * inv;
            info.centroid[2] = centroidAccum[2] * inv;
            index.components.push_back(info);
        }
    }

    return index;
}

PointComponentLabel LabelPointOnNavMesh(
    const dtNavMesh* navMesh,
    const ConnectivityIndex& index,
    const float* point,
    const float* extents)
{
    PointComponentLabel out;
    out.input[0] = point[0];
    out.input[1] = point[1];
    out.input[2] = point[2];

    if (!navMesh) {
        return out;
    }

    dtNavMeshQuery query;
    if (dtStatusFailed(query.init(navMesh, 2048))) {
        return out;
    }

    dtQueryFilter filter;
    const float* qExt = extents ? extents : kDefaultPointExtents;
    float snapped[3] = {0.0f, 0.0f, 0.0f};
    dtPolyRef ref = 0;

    const dtStatus st = query.findNearestPoly(point, qExt, &filter, &ref, snapped);
    if (dtStatusFailed(st) || ref == 0) {
        return out;
    }

    out.onMesh = true;
    out.polyRef = ref;
    out.snapped[0] = snapped[0];
    out.snapped[1] = snapped[1];
    out.snapped[2] = snapped[2];
    out.snapDistance = Distance3(out.input, out.snapped);

    const auto it = index.polyToComponent.find(ref);
    if (it != index.polyToComponent.end()) {
        out.componentId = it->second;
    }

    return out;
}

GapDiagnosis DiagnosePointGap(
    const dtNavMesh* navMesh,
    const ConnectivityIndex& index,
    const float* start,
    const float* end,
    int samples)
{
    GapDiagnosis d;

    d.start = LabelPointOnNavMesh(navMesh, index, start, kProbePointExtents);
    d.end = LabelPointOnNavMesh(navMesh, index, end, kProbePointExtents);

    if (!d.start.onMesh) {
        d.reasons.push_back({
            "start_off_mesh",
            0.0f,
            0.0f,
            "Start point could not be snapped to any navmesh polygon within search extents.",
        });
    }

    if (!d.end.onMesh) {
        d.reasons.push_back({
            "end_off_mesh",
            0.0f,
            0.0f,
            "End point could not be snapped to any navmesh polygon within search extents.",
        });
    }

    if (!d.start.onMesh || !d.end.onMesh) {
        return d;
    }

    d.sameComponent = (d.start.componentId >= 0 && d.start.componentId == d.end.componentId);
    if (d.sameComponent) {
        d.reasons.push_back({
            "same_component",
            0.0f,
            0.0f,
            "Both points are already in the same connected component.",
        });
        return d;
    }

    dtNavMeshQuery query;
    if (dtStatusFailed(query.init(navMesh, 4096))) {
        d.reasons.push_back({
            "query_init_failed",
            0.0f,
            0.0f,
            "Failed to initialize dtNavMeshQuery for local diagnostics.",
        });
        return d;
    }

    dtQueryFilter filter;
    const int clampedSamples = std::max(8, std::min(samples, 128));

    bool havePrev = false;
    float prev[3] = {0.0f, 0.0f, 0.0f};

    for (int i = 0; i <= clampedSamples; ++i) {
        const float t = static_cast<float>(i) / static_cast<float>(clampedSamples);
        float p[3] = {
            start[0] + (end[0] - start[0]) * t,
            start[1] + (end[1] - start[1]) * t,
            start[2] + (end[2] - start[2]) * t,
        };

        const PointComponentLabel lbl = LabelPointOnNavMesh(navMesh, index, p, kProbePointExtents);
        if (!lbl.onMesh) {
            continue;
        }

        ++d.sampledPoints;
        AppendUnique(d.sampledComponentPath, lbl.componentId);

        if (havePrev) {
            const float dx = lbl.snapped[0] - prev[0];
            const float dz = lbl.snapped[2] - prev[2];
            const float dy = std::fabs(lbl.snapped[1] - prev[1]);
            const float horizontal = std::sqrt(dx * dx + dz * dz);
            const float slopeDeg = std::atan2(dy, std::max(horizontal, 1e-4f)) * (180.0f / 3.1415926535f);
            d.maxSlopeDeg = std::max(d.maxSlopeDeg, slopeDeg);
            d.maxStepHeight = std::max(d.maxStepHeight, dy);
        }

        float hitPos[3] = {0.0f, 0.0f, 0.0f};
        float hitNorm[3] = {0.0f, 0.0f, 0.0f};
        float wallDist = 0.0f;
        const dtStatus wallStatus = query.findDistanceToWall(
            lbl.polyRef,
            lbl.snapped,
            10.0f,
            &filter,
            &wallDist,
            hitPos,
            hitNorm);

        if (dtStatusSucceed(wallStatus)) {
            if (d.minWallDistance < 0.0f) {
                d.minWallDistance = wallDist;
            } else {
                d.minWallDistance = std::min(d.minWallDistance, wallDist);
            }
        }

        prev[0] = lbl.snapped[0];
        prev[1] = lbl.snapped[1];
        prev[2] = lbl.snapped[2];
        havePrev = true;
    }

    if (d.sampledPoints == 0) {
        d.reasons.push_back({
            "no_local_coverage",
            0.0f,
            0.0f,
            "No nearby walkable samples were found between the two input points.",
        });
        return d;
    }

    if (d.maxSlopeDeg > 46.0f) {
        d.reasons.push_back({
            "slopeAngle",
            d.maxSlopeDeg,
            d.maxSlopeDeg,
            "Observed local vertical gradient suggests slope threshold may be too low in this corridor.",
        });
    }

    if (d.maxStepHeight > 0.5f) {
        d.reasons.push_back({
            "agentClimb",
            d.maxStepHeight,
            d.maxStepHeight,
            "Observed local step delta exceeds typical climb values used for these meshes.",
        });
    }

    if (d.minWallDistance >= 0.0f && d.minWallDistance < 0.35f) {
        d.reasons.push_back({
            "agentRadius",
            d.minWallDistance,
            d.minWallDistance,
            "Local clearance is narrow; larger erosion radius likely disconnects this corridor.",
        });
    }

    if (d.reasons.empty()) {
        d.reasons.push_back({
            "component_separation",
            0.0f,
            0.0f,
            "Points are in different components; no dominant static blocker was isolated from local sampling.",
        });
    }

    return d;
}

} // namespace debugnav
