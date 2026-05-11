#include "obj_preprocessing.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <unordered_set>
#include <unordered_map>
#include <vector>

// ── Internal helpers ──────────────────────────────────────────────────────────

namespace {

struct QuantizedKey {
    std::int32_t x;
    std::int32_t y;
    std::int32_t z;

    bool operator==(const QuantizedKey& o) const {
        return x == o.x && y == o.y && z == o.z;
    }
};

struct QuantizedKeyHash {
    std::size_t operator()(const QuantizedKey& k) const {
        // FNV-1a mix of three 32-bit values.
        std::size_t h = 2166136261u;
        auto mix = [&](std::uint32_t v) {
            h ^= static_cast<std::size_t>(v);
            h *= 16777619u;
        };
        mix(static_cast<std::uint32_t>(k.x));
        mix(static_cast<std::uint32_t>(k.y));
        mix(static_cast<std::uint32_t>(k.z));
        return h;
    }
};

struct DisjointSet {
    std::vector<int> parent;
    std::vector<int> rank;

    explicit DisjointSet(int n)
        : parent(static_cast<std::size_t>(n), 0),
          rank(static_cast<std::size_t>(n), 0)
    {
        for (int i = 0; i < n; ++i) parent[i] = i;
    }

    int Find(int x) {
        if (parent[x] != x) parent[x] = Find(parent[x]);
        return parent[x];
    }

    bool Union(int a, int b) {
        a = Find(a);
        b = Find(b);
        if (a == b) return false;
        if (rank[a] < rank[b]) std::swap(a, b);
        parent[b] = a;
        if (rank[a] == rank[b]) ++rank[a];
        return true;
    }
};

static std::uint64_t MakeEdgeKey(int a, int b)
{
    const std::uint32_t lo = static_cast<std::uint32_t>(std::min(a, b));
    const std::uint32_t hi = static_cast<std::uint32_t>(std::max(a, b));
    return (static_cast<std::uint64_t>(lo) << 32) | hi;
}

// Builds a mapping from each original vertex index to a canonical (welded)
// vertex index by quantizing positions to a grid of size `tolerance`.
static std::vector<int> BuildWeldMap(const ObjMesh& mesh, float tolerance)
{
    const int nVerts = static_cast<int>(mesh.verts.size() / 3);
    const float invTol = 1.0f / tolerance;

    std::vector<int> weldMap(static_cast<std::size_t>(nVerts), -1);
    std::unordered_map<QuantizedKey, int, QuantizedKeyHash> grid;
    grid.reserve(static_cast<std::size_t>(nVerts));

    int canonical = 0;
    for (int i = 0; i < nVerts; ++i) {
        const QuantizedKey key{
            static_cast<std::int32_t>(std::floor(mesh.verts[i * 3 + 0] * invTol)),
            static_cast<std::int32_t>(std::floor(mesh.verts[i * 3 + 1] * invTol)),
            static_cast<std::int32_t>(std::floor(mesh.verts[i * 3 + 2] * invTol)),
        };
        const auto [it, inserted] = grid.emplace(key, canonical);
        if (inserted) ++canonical;
        weldMap[i] = it->second;
    }

    return weldMap;
}

} // namespace

// ── WeldVertices ──────────────────────────────────────────────────────────────

WeldStats WeldVertices(ObjMesh& mesh, float tolerance)
{
    WeldStats stats{};
    stats.originalVerts = static_cast<int>(mesh.verts.size() / 3);

    const std::vector<int> weldMap = BuildWeldMap(mesh, tolerance);

    // Compute canonical vertex count.
    int nCanonical = 0;
    for (const int id : weldMap) nCanonical = std::max(nCanonical, id + 1);

    // Build compact vertex array: use the first vertex that maps to each
    // canonical slot.
    std::vector<float> newVerts(static_cast<std::size_t>(nCanonical) * 3, 0.0f);
    std::vector<bool>  filled(static_cast<std::size_t>(nCanonical), false);

    const int nVerts = stats.originalVerts;
    for (int i = 0; i < nVerts; ++i) {
        const int slot = weldMap[i];
        if (!filled[slot]) {
            newVerts[slot * 3 + 0] = mesh.verts[i * 3 + 0];
            newVerts[slot * 3 + 1] = mesh.verts[i * 3 + 1];
            newVerts[slot * 3 + 2] = mesh.verts[i * 3 + 2];
            filled[slot] = true;
        }
    }

    // Remap triangle indices.
    for (int& idx : mesh.tris) idx = weldMap[idx];

    mesh.verts = std::move(newVerts);

    stats.finalVerts  = nCanonical;
    stats.mergedVerts = stats.originalVerts - stats.finalVerts;
    return stats;
}

EdgeWeldStats WeldShortEdges(ObjMesh& mesh, float tolerance)
{
    EdgeWeldStats stats{};
    stats.originalVerts = static_cast<int>(mesh.verts.size() / 3);
    stats.originalTris  = static_cast<int>(mesh.tris.size() / 3);

    if (tolerance <= 0.0f || stats.originalVerts == 0 || stats.originalTris == 0) {
        stats.finalVerts = stats.originalVerts;
        stats.finalTris  = stats.originalTris;
        return stats;
    }

    const float tol2 = tolerance * tolerance;
    const int nVerts = stats.originalVerts;
    const int nTris  = stats.originalTris;

    DisjointSet dsu(nVerts);
    std::unordered_set<std::uint64_t> seenEdges;
    seenEdges.reserve(static_cast<std::size_t>(nTris) * 3);

    for (int t = 0; t < nTris; ++t) {
        const int i0 = mesh.tris[t * 3 + 0];
        const int i1 = mesh.tris[t * 3 + 1];
        const int i2 = mesh.tris[t * 3 + 2];
        const int edgeVerts[3][2] = { {i0, i1}, {i1, i2}, {i2, i0} };

        for (const auto& edge : edgeVerts) {
            const int a = edge[0];
            const int b = edge[1];
            if (a == b) continue;

            const std::uint64_t key = MakeEdgeKey(a, b);
            if (!seenEdges.insert(key).second) continue;

            const float dx = mesh.verts[a * 3 + 0] - mesh.verts[b * 3 + 0];
            const float dy = mesh.verts[a * 3 + 1] - mesh.verts[b * 3 + 1];
            const float dz = mesh.verts[a * 3 + 2] - mesh.verts[b * 3 + 2];
            const float d2 = dx * dx + dy * dy + dz * dz;
            if (d2 <= tol2 && dsu.Union(a, b)) {
                ++stats.collapsedEdges;
            }
        }
    }

    std::vector<int> rootToNew(static_cast<std::size_t>(nVerts), -1);
    std::vector<int> oldToNew(static_cast<std::size_t>(nVerts), -1);
    std::vector<float> newVerts;
    newVerts.reserve(mesh.verts.size());

    for (int i = 0; i < nVerts; ++i) {
        const int root = dsu.Find(i);
        int& mapped = rootToNew[root];
        if (mapped < 0) {
            mapped = static_cast<int>(newVerts.size() / 3);
            newVerts.push_back(mesh.verts[root * 3 + 0]);
            newVerts.push_back(mesh.verts[root * 3 + 1]);
            newVerts.push_back(mesh.verts[root * 3 + 2]);
        }
        oldToNew[i] = mapped;
    }

    std::vector<int> newTris;
    newTris.reserve(mesh.tris.size());
    for (int t = 0; t < nTris; ++t) {
        const int a = oldToNew[mesh.tris[t * 3 + 0]];
        const int b = oldToNew[mesh.tris[t * 3 + 1]];
        const int c = oldToNew[mesh.tris[t * 3 + 2]];
        if (a == b || b == c || c == a) {
            ++stats.removedDegenerateTris;
            continue;
        }
        newTris.push_back(a);
        newTris.push_back(b);
        newTris.push_back(c);
    }

    mesh.verts = std::move(newVerts);
    mesh.tris  = std::move(newTris);

    stats.finalVerts  = static_cast<int>(mesh.verts.size() / 3);
    stats.finalTris   = static_cast<int>(mesh.tris.size() / 3);
    stats.mergedVerts = stats.originalVerts - stats.finalVerts;
    return stats;
}

// ── NormalizeWinding ──────────────────────────────────────────────────────────

int NormalizeWinding(ObjMesh& mesh)
{
    const int nTris  = static_cast<int>(mesh.tris.size()) / 3;
    int       flipped = 0;

    for (int i = 0; i < nTris; ++i) {
        const int i0 = mesh.tris[i * 3 + 0];
        const int i1 = mesh.tris[i * 3 + 1];
        const int i2 = mesh.tris[i * 3 + 2];

        const float* v0 = &mesh.verts[i0 * 3];
        const float* v1 = &mesh.verts[i1 * 3];
        const float* v2 = &mesh.verts[i2 * 3];

        const float ax = v1[0] - v0[0];
        const float az = v1[2] - v0[2];
        const float bx = v2[0] - v0[0];
        const float bz = v2[2] - v0[2];
        // Y component of cross product (ax,0,az) × (bx,0,bz).
        const float ny = az * bx - ax * bz;
        if (ny < 0.0f) {
            std::swap(mesh.tris[i * 3 + 1], mesh.tris[i * 3 + 2]);
            ++flipped;
        }
    }

    return flipped;
}

// ── PruneSmallDisconnectedComponents ─────────────────────────────────────────

ComponentPruneStats PruneSmallDisconnectedComponents(ObjMesh& mesh,
                                                     int      minComponentTris)
{
    ComponentPruneStats stats{};
    const int nTris = static_cast<int>(mesh.tris.size()) / 3;

    if (nTris == 0 || minComponentTris <= 0) {
        stats.totalComponents  = (nTris > 0) ? 1 : 0;
        stats.keptComponents   = stats.totalComponents;
        stats.keptTris         = nTris;
        return stats;
    }

    // Build edge → triangle adjacency using the welded indices directly.
    // (Caller should invoke WeldVertices before this so near-coincident
    // vertices share the same index and therefore the same edges.)
    std::unordered_map<std::uint64_t, std::vector<int>> edgeToTris;
    edgeToTris.reserve(static_cast<std::size_t>(nTris) * 3);
    for (int t = 0; t < nTris; ++t) {
        const int i0 = mesh.tris[t * 3 + 0];
        const int i1 = mesh.tris[t * 3 + 1];
        const int i2 = mesh.tris[t * 3 + 2];
        edgeToTris[MakeEdgeKey(i0, i1)].push_back(t);
        edgeToTris[MakeEdgeKey(i1, i2)].push_back(t);
        edgeToTris[MakeEdgeKey(i2, i0)].push_back(t);
    }

    // BFS to label components.
    std::vector<char>            visited(static_cast<std::size_t>(nTris), 0);
    std::vector<std::vector<int>> components;

    for (int start = 0; start < nTris; ++start) {
        if (visited[start]) continue;

        std::vector<int> component;
        component.push_back(start);
        visited[start] = 1;

        for (std::size_t cursor = 0; cursor < component.size(); ++cursor) {
            const int t  = component[cursor];
            const int i0 = mesh.tris[t * 3 + 0];
            const int i1 = mesh.tris[t * 3 + 1];
            const int i2 = mesh.tris[t * 3 + 2];
            const std::uint64_t edges[3] = {
                MakeEdgeKey(i0, i1),
                MakeEdgeKey(i1, i2),
                MakeEdgeKey(i2, i0),
            };
            for (const std::uint64_t edge : edges) {
                const auto it = edgeToTris.find(edge);
                if (it == edgeToTris.end()) continue;
                for (const int nb : it->second) {
                    if (visited[nb]) continue;
                    visited[nb] = 1;
                    component.push_back(nb);
                }
            }
        }

        components.push_back(std::move(component));
    }

    stats.totalComponents = static_cast<int>(components.size());

    // Find largest component (always kept).
    int largestIdx  = 0;
    int largestSize = 0;
    for (int i = 0; i < static_cast<int>(components.size()); ++i) {
        if (static_cast<int>(components[i].size()) > largestSize) {
            largestSize = static_cast<int>(components[i].size());
            largestIdx  = i;
        }
    }

    // Mark which triangles to keep.
    std::vector<char> keepTri(static_cast<std::size_t>(nTris), 0);
    for (int i = 0; i < static_cast<int>(components.size()); ++i) {
        const int sz          = static_cast<int>(components[i].size());
        const bool keep       = (i == largestIdx) || (sz >= minComponentTris);
        if (keep) {
            ++stats.keptComponents;
            stats.keptTris += sz;
            for (const int t : components[i]) keepTri[t] = 1;
        } else {
            ++stats.prunedComponents;
            stats.prunedTris += sz;
        }
    }

    if (stats.prunedTris == 0) return stats;

    // Compact vertex and index arrays.
    const int nVerts = static_cast<int>(mesh.verts.size() / 3);
    std::vector<int>   vertexMap(static_cast<std::size_t>(nVerts), -1);
    std::vector<float> newVerts;
    std::vector<int>   newTris;
    newVerts.reserve(static_cast<std::size_t>(nVerts) * 3);
    newTris.reserve(static_cast<std::size_t>(stats.keptTris) * 3);

    for (int t = 0; t < nTris; ++t) {
        if (!keepTri[t]) continue;
        for (int c = 0; c < 3; ++c) {
            const int old = mesh.tris[t * 3 + c];
            int& neo       = vertexMap[old];
            if (neo < 0) {
                neo = static_cast<int>(newVerts.size() / 3);
                newVerts.push_back(mesh.verts[old * 3 + 0]);
                newVerts.push_back(mesh.verts[old * 3 + 1]);
                newVerts.push_back(mesh.verts[old * 3 + 2]);
            }
            newTris.push_back(neo);
        }
    }

    mesh.verts = std::move(newVerts);
    mesh.tris  = std::move(newTris);
    return stats;
}
