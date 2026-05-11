#pragma once

#include "../mesh_io.h"
#include "navmesh_generation_common.h"

class dtNavMesh;

bool GenerateTiledNavMesh(const ObjMesh&     mesh,
                          const BuildConfig& cfg,
                          float              cellSize,
                          int                navMinComponentPolys,
                          dtNavMesh**        outNavMesh);
