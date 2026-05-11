#pragma once

#include "mesh_io.h"

// ── Vertex welding ────────────────────────────────────────────────────────────

struct WeldStats {
    int originalVerts = 0;
    int finalVerts    = 0;
    int mergedVerts   = 0;  // originalVerts - finalVerts
};

// Merges vertices whose positions are within `tolerance` game units of each
// other by quantizing coordinates to a grid of that size. Rebuilds the index
// buffer in-place so all triangles reference the canonical vertex for each
// position bucket. The vertex array shrinks accordingly.
//
// A tolerance of 0.01 (1 cm) removes floating-point near-duplicates and
// sub-centimetre cracks introduced by FFXI's collision export without merging
// intentionally distinct geometry.
WeldStats WeldVertices(ObjMesh& mesh, float tolerance = 0.01f);

struct EdgeWeldStats {
    int originalVerts         = 0;
    int finalVerts            = 0;
    int mergedVerts           = 0;
    int collapsedEdges        = 0;
    int originalTris          = 0;
    int finalTris             = 0;
    int removedDegenerateTris = 0;
};

// Collapses short edges whose endpoint distance is <= `tolerance` by merging
// both vertices to a shared representative. This can bridge tiny cracks that
// remain after position-quantized vertex welding. Degenerate triangles created
// by merges are removed and the mesh is compacted.
//
// A tolerance of 0 disables this step.
EdgeWeldStats WeldShortEdges(ObjMesh& mesh, float tolerance = 0.0f);

// ── Winding normalization ─────────────────────────────────────────────────────

// Flips individual triangles whose XZ-plane normal component (Y in game space)
// is negative so that all triangles nominally face upward. Mixed source winding
// is common in extracted FFXI collision meshes.
// Returns the number of triangles that were flipped.
int NormalizeWinding(ObjMesh& mesh);

// ── Small component pruning ───────────────────────────────────────────────────

struct ComponentPruneStats {
    int totalComponents   = 0;
    int keptComponents    = 0;
    int prunedComponents  = 0;
    int keptTris          = 0;
    int prunedTris        = 0;
};

// Finds connected components in the triangle mesh using edge-sharing adjacency
// (after vertex positions are canonicalized by exact bit-match, so call
// WeldVertices first). Removes components with fewer than `minComponentTris`
// triangles. The largest component is always kept regardless of its size.
// Compacts the vertex and index arrays in-place.
ComponentPruneStats PruneSmallDisconnectedComponents(ObjMesh& mesh,
                                                     int minComponentTris);
