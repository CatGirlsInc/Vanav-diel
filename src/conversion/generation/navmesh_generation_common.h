#pragma once

#include "../mesh_io.h"

#include <Recast.h>

#include <cstdint>
#include <vector>

struct BuildConfig {
    float cs          = 0.40f;
    float ch          = 0.20f;
    float agentHeight = 1.80f;
    float agentRadius = 0.20f;
    float agentClimb  = 0.50f;
    float slopeAngle  = 46.0f;

    int   regionMinSize      = 8;
    int   regionMergeSize    = 20;
    float edgeMaxLen         = 12.0f;
    float edgeMaxError       = 1.3f;
    int   vertsPerPoly       = 6;
    float detailSampleDist   = 6.0f;
    float detailSampleMaxErr = 1.0f;

    float weldTolerance     = 0.01f;
    float edgeWeldTolerance = 0.0f;
    int   minComponentTris  = 0;
    int   minNavComponentPolys = 11;

    int tileSizeCells   = 160;
    int maxTiles        = 4096;
    int maxPolysPerTile = 4096;
};

struct NavComponentPruneStats {
    int totalComponents  = 0;
    int keptComponents   = 0;
    int prunedComponents = 0;
    int keptPolys        = 0;
    int prunedPolys      = 0;
};

void ComputeMeshBounds(const ObjMesh& mesh, float pad, float bmin[3], float bmax[3]);

NavComponentPruneStats PruneSmallNavPolyComponents(rcPolyMesh& pmesh,
                                                   int         minComponentPolys);

std::vector<int> BuildNavComponentPruneThresholds(int baseline);
