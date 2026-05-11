#include "navmesh_debug_tool.h"

#include <DetourAlloc.h>

#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace debugtool {

void PrintDebugUsage(const char* exeName, const char* meshLabel, std::ostream& out) {
    out
        << "Usage:\n"
        << "  " << exeName << " <" << meshLabel << "> --components [--json] [--json-out path]\n"
        << "  " << exeName << " <" << meshLabel << "> --point x,y,z [--point x,y,z ...] [--json]\n"
        << "  " << exeName << " <" << meshLabel << "> --from x,y,z --to x,y,z --diagnose-gap [--json]\n\n"
        << "Options:\n"
        << "  --components         Enumerate connected polygon components\n"
        << "  --point x,y,z        Label coordinate by snapped polygon + component id (repeatable)\n"
        << "  --from x,y,z         Diagnose start coordinate\n"
        << "  --to x,y,z           Diagnose end coordinate\n"
        << "  --diagnose-gap       Run local blocker diagnosis for from/to pair\n"
        << "  --json               Emit JSON to stdout\n"
        << "  --json-out <path>    Also write JSON to file\n";
}

bool ParseDebugArgs(
    int argc,
    char** argv,
    const char* meshLabel,
    const ParseVec3Fn& parseVec3,
    DebugArgs* out,
    std::ostream& err)
{
    if (argc < 2) {
        err << "Missing required <" << meshLabel << "> mesh path.\n";
        return false;
    }

    if (argv[1][0] == '-') {
        err << "First argument must be <" << meshLabel << "> mesh path; options come after it.\n";
        return false;
    }

    out->meshPath = argv[1];
    bool diagnoseGap = false;

    for (int i = 2; i < argc; ++i) {
        const std::string arg = argv[i];
        if (arg == "--components") {
            out->components = true;
        } else if (arg == "--json") {
            out->json = true;
        } else if (arg == "--json-out" && i + 1 < argc) {
            out->jsonOutPath = argv[++i];
        } else if (arg == "--point" && i + 1 < argc) {
            std::array<float, 3> p{};
            if (!parseVec3(argv[++i], p.data())) {
                err << "Failed to parse --point value: " << argv[i] << '\n';
                return false;
            }
            out->points.push_back(p);
        } else if (arg == "--from" && i + 1 < argc) {
            if (!parseVec3(argv[++i], out->from)) {
                err << "Failed to parse --from value: " << argv[i] << '\n';
                return false;
            }
            out->hasFrom = true;
        } else if (arg == "--to" && i + 1 < argc) {
            if (!parseVec3(argv[++i], out->to)) {
                err << "Failed to parse --to value: " << argv[i] << '\n';
                return false;
            }
            out->hasTo = true;
        } else if (arg == "--diagnose-gap") {
            diagnoseGap = true;
        } else {
            err << "Unknown argument: " << arg << '\n';
            return false;
        }
    }

    if (out->points.empty() && !out->components && !diagnoseGap) {
        out->components = true;
    }

    if (diagnoseGap && (!out->hasFrom || !out->hasTo)) {
        err << "--diagnose-gap requires --from and --to.\n";
        return false;
    }

    return true;
}

std::string Vec3Text(const float v[3]) {
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(3)
        << "(" << v[0] << ", " << v[1] << ", " << v[2] << ")";
    return oss.str();
}

std::string BuildJsonReport(
    const std::filesystem::path& meshPath,
    const debugnav::ConnectivityIndex& index,
    bool includeComponents,
    const std::vector<debugnav::PointComponentLabel>& pointLabels,
    const std::optional<debugnav::GapDiagnosis>& diagnosis)
{
    std::ostringstream out;
    out << std::fixed << std::setprecision(3);
    out << "{\n";
    out << "  \"mesh\": \"" << meshPath.string() << "\",\n";
    out << "  \"walkable_poly_count\": " << index.walkablePolyCount << ",\n";
    out << "  \"component_count\": " << index.components.size() << ",\n";

    out << "  \"components\": ";
    if (!includeComponents) {
        out << "null,\n";
    } else {
        out << "[\n";
        for (std::size_t i = 0; i < index.components.size(); ++i) {
            const auto& c = index.components[i];
            out << "    {\"id\": " << c.componentId
                << ", \"poly_count\": " << c.polyCount
                << ", \"bounds_min\": [" << c.minBounds[0] << ", " << c.minBounds[1] << ", " << c.minBounds[2] << "]"
                << ", \"bounds_max\": [" << c.maxBounds[0] << ", " << c.maxBounds[1] << ", " << c.maxBounds[2] << "]"
                << ", \"centroid\": [" << c.centroid[0] << ", " << c.centroid[1] << ", " << c.centroid[2] << "]"
                << "}";
            out << (i + 1 < index.components.size() ? ",\n" : "\n");
        }
        out << "  ],\n";
    }

    out << "  \"point_labels\": [\n";
    for (std::size_t i = 0; i < pointLabels.size(); ++i) {
        const auto& p = pointLabels[i];
        out << "    {\"input\": [" << p.input[0] << ", " << p.input[1] << ", " << p.input[2] << "]"
            << ", \"on_mesh\": " << (p.onMesh ? "true" : "false")
            << ", \"snapped\": [" << p.snapped[0] << ", " << p.snapped[1] << ", " << p.snapped[2] << "]"
            << ", \"component_id\": " << p.componentId
            << ", \"snap_distance\": " << p.snapDistance
            << "}";
        out << (i + 1 < pointLabels.size() ? ",\n" : "\n");
    }
    out << "  ]";

    if (diagnosis.has_value()) {
        const auto& d = *diagnosis;
        out << ",\n  \"diagnose_gap\": {\n";
        out << "    \"same_component\": " << (d.sameComponent ? "true" : "false") << ",\n";
        out << "    \"sampled_points\": " << d.sampledPoints << ",\n";
        out << "    \"max_slope_deg\": " << d.maxSlopeDeg << ",\n";
        out << "    \"max_step_height\": " << d.maxStepHeight << ",\n";
        out << "    \"min_wall_distance\": " << d.minWallDistance << ",\n";
        out << "    \"sampled_component_path\": [";
        for (std::size_t i = 0; i < d.sampledComponentPath.size(); ++i) {
            out << d.sampledComponentPath[i];
            if (i + 1 < d.sampledComponentPath.size()) out << ", ";
        }
        out << "],\n";
        out << "    \"reasons\": [\n";
        for (std::size_t i = 0; i < d.reasons.size(); ++i) {
            const auto& r = d.reasons[i];
            out << "      {\"kind\": \"" << r.kind
                << "\", \"measured\": " << r.measured
                << ", \"suggested_threshold\": " << r.suggestedThreshold
                << ", \"detail\": \"" << r.detail << "\"}";
            out << (i + 1 < d.reasons.size() ? ",\n" : "\n");
        }
        out << "    ]\n";
        out << "  }\n";
    } else {
        out << "\n";
    }

    out << "}\n";
    return out.str();
}

int RunDebugTool(
    const DebugArgs& args,
    const LoadNavMeshFn& loadNavMesh,
    std::ostream& out,
    std::ostream& err)
{
    if (!std::filesystem::exists(args.meshPath)) {
        err << "Input does not exist: " << args.meshPath << '\n';
        return 1;
    }

    dtNavMesh* navMesh = nullptr;
    if (!loadNavMesh(args.meshPath, &navMesh)) {
        return 1;
    }

    const debugnav::ConnectivityIndex index = debugnav::BuildConnectivityIndex(navMesh);

    std::vector<debugnav::PointComponentLabel> labels;
    labels.reserve(args.points.size());
    for (const auto& p : args.points) {
        labels.push_back(debugnav::LabelPointOnNavMesh(navMesh, index, p.data()));
    }

    std::optional<debugnav::GapDiagnosis> diagnosis;
    if (args.hasFrom && args.hasTo) {
        diagnosis = debugnav::DiagnosePointGap(navMesh, index, args.from, args.to);
    }

    if (!args.json) {
        out << "Mesh: " << args.meshPath << '\n';
        out << "Walkable polys: " << index.walkablePolyCount << '\n';
        out << "Components: " << index.components.size() << "\n\n";

        if (args.components) {
            out << "== Connected Components ==\n";
            for (const auto& c : index.components) {
                out << "  [component " << c.componentId << "]"
                    << " polys=" << c.polyCount
                    << " bounds_min=" << Vec3Text(c.minBounds)
                    << " bounds_max=" << Vec3Text(c.maxBounds)
                    << " centroid=" << Vec3Text(c.centroid)
                    << '\n';
            }
            out << '\n';
        }

        if (!labels.empty()) {
            out << "== Point Component Labels ==\n";
            for (const auto& p : labels) {
                out << "  point " << Vec3Text(p.input) << " -> ";
                if (!p.onMesh) {
                    out << "OFF_MESH\n";
                    continue;
                }
                out << "component=" << p.componentId
                    << " snapped=" << Vec3Text(p.snapped)
                    << " snap_dist=" << std::fixed << std::setprecision(3) << p.snapDistance
                    << " poly_ref=" << p.polyRef
                    << '\n';
            }
            out << '\n';
        }

        if (diagnosis.has_value()) {
            const auto& d = *diagnosis;
            out << "== Diagnose Gap ==\n";
            out << "  start_component=" << d.start.componentId
                << " end_component=" << d.end.componentId
                << " same_component=" << (d.sameComponent ? "yes" : "no") << '\n';
            out << "  sampled_points=" << d.sampledPoints
                << " max_slope_deg=" << std::fixed << std::setprecision(2) << d.maxSlopeDeg
                << " max_step_height=" << d.maxStepHeight
                << " min_wall_distance=" << d.minWallDistance
                << '\n';
            out << "  sampled_component_path=";
            for (std::size_t i = 0; i < d.sampledComponentPath.size(); ++i) {
                out << d.sampledComponentPath[i];
                if (i + 1 < d.sampledComponentPath.size()) out << " -> ";
            }
            out << '\n';
            out << "  reasons:\n";
            for (const auto& r : d.reasons) {
                out << "    - " << r.kind
                    << " measured=" << r.measured
                    << " suggested_threshold=" << r.suggestedThreshold
                    << " detail=\"" << r.detail << "\"\n";
            }
        }
    }

    if (args.json || !args.jsonOutPath.empty()) {
        const std::string json = BuildJsonReport(args.meshPath, index, args.components, labels, diagnosis);
        if (args.json) {
            out << json;
        }
        if (!args.jsonOutPath.empty()) {
            std::ofstream jsonOut(args.jsonOutPath);
            if (!jsonOut) {
                err << "Failed to write JSON output: " << args.jsonOutPath << '\n';
                dtFreeNavMesh(navMesh);
                return 1;
            }
            jsonOut << json;
        }
    }

    dtFreeNavMesh(navMesh);
    return 0;
}

} // namespace debugtool
