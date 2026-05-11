#pragma once
// Lightweight navmesh path-query helper for unit tests.
// Depends only on Detour; does not pull in zone_route or any FFXIPathfinding code.

#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>
#include <DetourAlloc.h>

#include <vector>

struct TestWaypoint { float x, y, z; };

// Returns the straight-path waypoints between `from` and `to` using `nav`.
// Returns an empty vector when either endpoint has no nearby polygon or the
// positions are not mutually reachable.
inline std::vector<TestWaypoint> QueryPath(const dtNavMesh* nav,
                                           const float*     from,
                                           const float*     to)
{
    static constexpr int   kMaxSearchNodes = 16384;
    static constexpr int   kMaxPolys      = 512;
    static constexpr int   kMaxStraight   = 512;
    static constexpr float kHalfExtents[3] = {10.0f, 25.0f, 10.0f};

    dtNavMeshQuery* q = dtAllocNavMeshQuery();
    if (!q) return {};
    if (dtStatusFailed(q->init(nav, kMaxSearchNodes))) { dtFreeNavMeshQuery(q); return {}; }

    dtQueryFilter filter;
    dtPolyRef startRef = 0, endRef = 0;
    float snappedStart[3], snappedEnd[3];

    if (dtStatusFailed(q->findNearestPoly(from, kHalfExtents, &filter,
                                          &startRef, snappedStart))
            || startRef == 0) {
        dtFreeNavMeshQuery(q); return {};
    }
    if (dtStatusFailed(q->findNearestPoly(to, kHalfExtents, &filter,
                                          &endRef, snappedEnd))
            || endRef == 0) {
        dtFreeNavMeshQuery(q); return {};
    }

    dtPolyRef polys[kMaxPolys];
    int polyCount = 0;
    if (dtStatusFailed(q->findPath(startRef, endRef, snappedStart, snappedEnd,
                                   &filter, polys, &polyCount, kMaxPolys))
            || polyCount == 0
            || polys[polyCount - 1] != endRef) {
        dtFreeNavMeshQuery(q); return {};
    }

    float            straightPath[kMaxStraight * 3];
    unsigned char    straightFlags[kMaxStraight];
    dtPolyRef        straightRefs[kMaxStraight];
    int              straightCount = 0;
    q->findStraightPath(snappedStart, snappedEnd, polys, polyCount,
                        straightPath, straightFlags, straightRefs,
                        &straightCount, kMaxStraight);
    dtFreeNavMeshQuery(q);

    std::vector<TestWaypoint> result;
    result.reserve(static_cast<std::size_t>(straightCount));
    for (int i = 0; i < straightCount; ++i)
        result.push_back({straightPath[i*3], straightPath[i*3+1], straightPath[i*3+2]});
    return result;
}
