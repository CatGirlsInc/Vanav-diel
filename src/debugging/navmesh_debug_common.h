#pragma once

#include <DetourNavMesh.h>

#include <string>
#include <unordered_map>
#include <vector>

namespace debugnav {

struct ComponentInfo {
    int componentId = -1;
    int polyCount = 0;
    float minBounds[3] = {0.0f, 0.0f, 0.0f};
    float maxBounds[3] = {0.0f, 0.0f, 0.0f};
    float centroid[3] = {0.0f, 0.0f, 0.0f};
};

struct ConnectivityIndex {
    std::unordered_map<dtPolyRef, int> polyToComponent;
    std::vector<ComponentInfo> components;
    int walkablePolyCount = 0;
};

struct PointComponentLabel {
    bool onMesh = false;
    float input[3] = {0.0f, 0.0f, 0.0f};
    float snapped[3] = {0.0f, 0.0f, 0.0f};
    dtPolyRef polyRef = 0;
    int componentId = -1;
    float snapDistance = 0.0f;
};

struct BlockerReason {
    std::string kind;
    float measured = 0.0f;
    float suggestedThreshold = 0.0f;
    std::string detail;
};

struct GapDiagnosis {
    PointComponentLabel start;
    PointComponentLabel end;
    bool sameComponent = false;
    int sampledPoints = 0;
    float maxSlopeDeg = 0.0f;
    float maxStepHeight = 0.0f;
    float minWallDistance = -1.0f;
    std::vector<int> sampledComponentPath;
    std::vector<BlockerReason> reasons;
};

ConnectivityIndex BuildConnectivityIndex(const dtNavMesh* navMesh);

PointComponentLabel LabelPointOnNavMesh(
    const dtNavMesh* navMesh,
    const ConnectivityIndex& index,
    const float* point,
    const float* extents = nullptr);

GapDiagnosis DiagnosePointGap(
    const dtNavMesh* navMesh,
    const ConnectivityIndex& index,
    const float* start,
    const float* end,
    int samples = 32);

} // namespace debugnav
