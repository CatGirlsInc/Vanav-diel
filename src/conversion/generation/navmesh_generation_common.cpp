#include "navmesh_generation_common.h"

#include <algorithm>
#include <limits>
#include <unordered_map>

namespace {

std::uint64_t MakePolyEdgeKey(unsigned int a, unsigned int b)
{
    if (a > b) std::swap(a, b);
    return (static_cast<std::uint64_t>(a) << 32) | static_cast<std::uint64_t>(b);
}

} // namespace

void ComputeMeshBounds(const ObjMesh& mesh, float pad, float bmin[3], float bmax[3])
{
    const float* verts = mesh.verts.data();
    const int nVerts = static_cast<int>(mesh.verts.size()) / 3;

    bmin[0] = std::numeric_limits<float>::max();
    bmin[1] = std::numeric_limits<float>::max();
    bmin[2] = std::numeric_limits<float>::max();
    bmax[0] = -std::numeric_limits<float>::max();
    bmax[1] = -std::numeric_limits<float>::max();
    bmax[2] = -std::numeric_limits<float>::max();

    for (int i = 0; i < nVerts; ++i) {
        bmin[0] = std::min(bmin[0], verts[i * 3 + 0]);
        bmin[1] = std::min(bmin[1], verts[i * 3 + 1]);
        bmin[2] = std::min(bmin[2], verts[i * 3 + 2]);
        bmax[0] = std::max(bmax[0], verts[i * 3 + 0]);
        bmax[1] = std::max(bmax[1], verts[i * 3 + 1]);
        bmax[2] = std::max(bmax[2], verts[i * 3 + 2]);
    }

    for (int i = 0; i < 3; ++i) {
        bmin[i] -= pad;
        bmax[i] += pad;
    }
}

NavComponentPruneStats PruneSmallNavPolyComponents(rcPolyMesh& pmesh,
                                                   int         minComponentPolys)
{
    NavComponentPruneStats stats{};
    if (minComponentPolys <= 0 || pmesh.npolys <= 0 || pmesh.nvp <= 0) {
        return stats;
    }

    std::vector<int> activePolys;
    activePolys.reserve(static_cast<std::size_t>(pmesh.npolys));
    for (int pi = 0; pi < pmesh.npolys; ++pi) {
        if (pmesh.areas[pi] != RC_NULL_AREA) activePolys.push_back(pi);
    }
    if (activePolys.empty()) return stats;

    std::unordered_map<std::uint64_t, std::vector<int>> edgeToPolys;
    edgeToPolys.reserve(activePolys.size() * 3u);

    for (const int pi : activePolys) {
        const unsigned short* poly = &pmesh.polys[pi * 2 * pmesh.nvp];
        int vertCount = 0;
        while (vertCount < pmesh.nvp && poly[vertCount] != RC_MESH_NULL_IDX) ++vertCount;
        if (vertCount < 3) continue;

        for (int vi = 0; vi < vertCount; ++vi) {
            const unsigned int a = poly[vi];
            const unsigned int b = poly[(vi + 1) % vertCount];
            edgeToPolys[MakePolyEdgeKey(a, b)].push_back(pi);
        }
    }

    std::vector<std::vector<int>> adjacency(static_cast<std::size_t>(pmesh.npolys));
    for (const auto& kv : edgeToPolys) {
        const std::vector<int>& polys = kv.second;
        if (polys.size() < 2) continue;
        for (std::size_t i = 0; i < polys.size(); ++i) {
            for (std::size_t j = i + 1; j < polys.size(); ++j) {
                adjacency[polys[i]].push_back(polys[j]);
                adjacency[polys[j]].push_back(polys[i]);
            }
        }
    }

    std::vector<char> visited(static_cast<std::size_t>(pmesh.npolys), 0);
    std::vector<std::vector<int>> components;

    for (const int start : activePolys) {
        if (visited[start]) continue;
        std::vector<int> component;
        component.push_back(start);
        visited[start] = 1;

        for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
            const int cur = component[cursor];
            for (const int nb : adjacency[cur]) {
                if (visited[nb]) continue;
                visited[nb] = 1;
                component.push_back(nb);
            }
        }

        components.push_back(std::move(component));
    }

    stats.totalComponents = static_cast<int>(components.size());
    if (components.empty()) return stats;

    int largestIdx = 0;
    int largestSz = 0;
    for (int i = 0; i < static_cast<int>(components.size()); ++i) {
        const int sz = static_cast<int>(components[i].size());
        if (sz > largestSz) {
            largestSz = sz;
            largestIdx = i;
        }
    }

    for (int i = 0; i < static_cast<int>(components.size()); ++i) {
        const int sz = static_cast<int>(components[i].size());
        const bool keep = (i == largestIdx) || (sz >= minComponentPolys);
        if (keep) {
            ++stats.keptComponents;
            stats.keptPolys += sz;
            continue;
        }

        ++stats.prunedComponents;
        stats.prunedPolys += sz;
        for (const int pi : components[i]) {
            pmesh.areas[pi] = RC_NULL_AREA;
        }
    }

    return stats;
}

std::vector<int> BuildNavComponentPruneThresholds(int baseline)
{
    std::vector<int> thresholds;
    thresholds.push_back(std::max(0, baseline));
    for (const int threshold : {24, 32, 48, 64, 96, 128}) {
        if (threshold > thresholds.back()) {
            thresholds.push_back(threshold);
        }
    }
    return thresholds;
}
