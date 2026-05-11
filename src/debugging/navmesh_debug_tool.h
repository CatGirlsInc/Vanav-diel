#pragma once

#include <DetourNavMesh.h>

#include <array>
#include <filesystem>
#include <functional>
#include <iosfwd>
#include <optional>
#include <string>
#include <vector>

#include "debugging/navmesh_debug_common.h"

namespace debugtool {

struct DebugArgs {
    std::filesystem::path meshPath;
    bool components = false;
    bool json = false;
    std::filesystem::path jsonOutPath;
    bool hasFrom = false;
    bool hasTo = false;
    float from[3] = {0.0f, 0.0f, 0.0f};
    float to[3] = {0.0f, 0.0f, 0.0f};
    std::vector<std::array<float, 3>> points;
};

using ParseVec3Fn = std::function<bool(const std::string&, float[3])>;
using LoadNavMeshFn = std::function<bool(const std::filesystem::path&, dtNavMesh**)>;

void PrintDebugUsage(const char* exeName, const char* meshLabel, std::ostream& out);

bool ParseDebugArgs(
    int argc,
    char** argv,
    const char* meshLabel,
    const ParseVec3Fn& parseVec3,
    DebugArgs* out,
    std::ostream& err);

std::string BuildJsonReport(
    const std::filesystem::path& meshPath,
    const debugnav::ConnectivityIndex& index,
    bool includeComponents,
    const std::vector<debugnav::PointComponentLabel>& pointLabels,
    const std::optional<debugnav::GapDiagnosis>& diagnosis);

std::string Vec3Text(const float v[3]);

int RunDebugTool(
    const DebugArgs& args,
    const LoadNavMeshFn& loadNavMesh,
    std::ostream& out,
    std::ostream& err);

} // namespace debugtool
