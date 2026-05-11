#include "viz.h"
#include "debugging/navmesh_debug_common.h"

#include <raylib.h>
#include <raymath.h>
#include <rlgl.h>

#include <DetourNavMesh.h>
#include <DetourAlloc.h>

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <vector>

namespace {

// ─── Zone transition JSON parsing (private helpers) ───────────────────────────

std::optional<Vector3> ParseVec3ArrayAt(const std::string& s, std::size_t keyPos) {
    const std::size_t lb = s.find('[', keyPos);
    if (lb == std::string::npos) return std::nullopt;
    const std::size_t rb = s.find(']', lb);
    if (rb == std::string::npos) return std::nullopt;
    const std::string body = s.substr(lb + 1, rb - lb - 1);

    float x = 0.0f, y = 0.0f, z = 0.0f;
    if (std::sscanf(body.c_str(), " %f , %f , %f ", &x, &y, &z) != 3) {
        return std::nullopt;
    }
    return Vector3{x, y, z};
}

std::string ParseJsonStringFieldAfter(const std::string& s, const std::string& key, std::size_t startPos) {
    const std::size_t keyPos = s.find(key, startPos);
    if (keyPos == std::string::npos) return {};
    const std::size_t q1 = s.find('"', keyPos + key.size());
    if (q1 == std::string::npos) return {};
    const std::size_t q2 = s.find('"', q1 + 1);
    if (q2 == std::string::npos) return {};
    return s.substr(q1 + 1, q2 - q1 - 1);
}

std::filesystem::path GuessConnectionsPath(const std::filesystem::path& zoneBinPath) {
    const std::string file = zoneBinPath.stem().string() + "_connections.json";
    std::error_code ec;

    const std::filesystem::path direct = zoneBinPath.parent_path() / file;
    if (std::filesystem::exists(direct, ec)) return direct;

    const std::filesystem::path repoStyle = zoneBinPath.parent_path().parent_path() / "OBJs" / file;
    if (std::filesystem::exists(repoStyle, ec)) return repoStyle;

    return {};
}

// ─── Raycasting (private helper) ──────────────────────────────────────────────

// Double-sided Möller–Trumbore ray-triangle intersection.
// Returns true and sets *outT to the distance along the ray on hit.
bool RayHitsTriangle(
    Vector3 ro, Vector3 rd,
    Vector3 v0, Vector3 v1, Vector3 v2,
    float* outT)
{
    constexpr float kEps = 1e-7f;
    const Vector3 e1 = Vector3Subtract(v1, v0);
    const Vector3 e2 = Vector3Subtract(v2, v0);
    const Vector3 h  = Vector3CrossProduct(rd, e2);
    const float   a  = Vector3DotProduct(e1, h);
    if (fabsf(a) < kEps) return false;
    const float   f  = 1.0f / a;
    const Vector3 s  = Vector3Subtract(ro, v0);
    const float   u  = f * Vector3DotProduct(s, h);
    if (u < 0.0f || u > 1.0f) return false;
    const Vector3 q  = Vector3CrossProduct(s, e1);
    const float   v  = f * Vector3DotProduct(rd, q);
    if (v < 0.0f || u + v > 1.0f) return false;
    const float   t  = f * Vector3DotProduct(e2, q);
    if (t < kEps) return false;
    *outT = t;
    return true;
}

// Returns the closest hit position on the nav mesh, or nullopt if no triangle was hit.
struct RaycastHit {
    Vector3 position;
    int triangleIndex = -1;
};

Color ComponentColorForId(int componentId) {
    if (componentId < 0) {
        return Color{80, 190, 120, 255};
    }
    const int hueSeed = (componentId * 47) % 360;
    return ColorFromHSV(static_cast<float>(hueSeed), 0.62f, 0.88f);
}

std::optional<RaycastHit> RaycastNavMesh(
    const std::vector<Triangle>& triangles,
    Vector3 ro, Vector3 rd)
{
    float bestT = std::numeric_limits<float>::max();
    int hitTri = -1;
    for (std::size_t i = 0; i < triangles.size(); ++i) {
        const Triangle& tri = triangles[i];
        float t;
        if (RayHitsTriangle(ro, rd, tri.a, tri.b, tri.c, &t) && t < bestT) {
            bestT = t;
            hitTri = static_cast<int>(i);
        }
    }
    if (hitTri < 0) return std::nullopt;

    RaycastHit out;
    out.position = Vector3Add(ro, Vector3Scale(rd, bestT));
    out.triangleIndex = hitTri;
    return out;
}

} // namespace

// ─── NavMesh geometry helpers ──────────────────────────────────────────────────

std::vector<Triangle> ExtractTriangles(const dtNavMesh* navMesh) {
    std::vector<Triangle> triangles;
    const debugnav::ConnectivityIndex connectivity = debugnav::BuildConnectivityIndex(navMesh);

    const int maxTiles = navMesh->getMaxTiles();
    for (int tileIndex = 0; tileIndex < maxTiles; ++tileIndex) {
        const dtMeshTile* tile = navMesh->getTile(tileIndex);
        if (!tile || !tile->header) {
            continue;
        }

        for (int polyIndex = 0; polyIndex < tile->header->polyCount; ++polyIndex) {
            const dtPoly* poly = &tile->polys[polyIndex];
            if (poly->flags == 0 ||
                poly->getType() == DT_POLYTYPE_OFFMESH_CONNECTION ||
                poly->vertCount < 3) {
                continue;
            }

            const dtPolyRef polyRef = navMesh->getPolyRefBase(tile) | static_cast<dtPolyRef>(polyIndex);
            int componentId = -1;
            const auto cIt = connectivity.polyToComponent.find(polyRef);
            if (cIt != connectivity.polyToComponent.end()) {
                componentId = cIt->second;
            }

            const unsigned int first = poly->verts[0];
            const float* firstVert = &tile->verts[first * 3];
            const Vector3 a{firstVert[0], firstVert[1], firstVert[2]};

            for (unsigned int i = 2; i < poly->vertCount; ++i) {
                const unsigned int bIndex = poly->verts[i - 1];
                const unsigned int cIndex = poly->verts[i];

                const float* bVert = &tile->verts[bIndex * 3];
                const float* cVert = &tile->verts[cIndex * 3];

                triangles.push_back({
                    a,
                    Vector3{bVert[0], bVert[1], bVert[2]},
                    Vector3{cVert[0], cVert[1], cVert[2]},
                    componentId,
                });
            }
        }
    }

    return triangles;
}

bool ComputeBounds(const std::vector<Triangle>& triangles, Vector3* outMin, Vector3* outMax) {
    if (triangles.empty()) {
        return false;
    }

    Vector3 minV{
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
    };

    Vector3 maxV{
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
        -std::numeric_limits<float>::max(),
    };

    auto includePoint = [&](const Vector3& p) {
        minV.x = std::min(minV.x, p.x);
        minV.y = std::min(minV.y, p.y);
        minV.z = std::min(minV.z, p.z);
        maxV.x = std::max(maxV.x, p.x);
        maxV.y = std::max(maxV.y, p.y);
        maxV.z = std::max(maxV.z, p.z);
    };

    for (const Triangle& tri : triangles) {
        includePoint(tri.a);
        includePoint(tri.b);
        includePoint(tri.c);
    }

    *outMin = minV;
    *outMax = maxV;
    return true;
}

// ─── Zone browser helpers ──────────────────────────────────────────────────────

std::vector<ZoneEntry> ScanZoneDir(const std::filesystem::path& dir) {
    std::vector<ZoneEntry> zones;
    std::error_code ec;
    if (!std::filesystem::is_directory(dir, ec)) return zones;
    for (const auto& entry : std::filesystem::directory_iterator(dir, ec)) {
        if (entry.path().extension() == ".bin") {
            zones.push_back({entry.path().stem().string(), entry.path()});
        }
    }
    std::sort(zones.begin(), zones.end(),
              [](const ZoneEntry& a, const ZoneEntry& b) { return a.name < b.name; });
    return zones;
}

std::vector<ZoneTransition> LoadZoneTransitions(const std::filesystem::path& zoneBinPath) {
    std::vector<ZoneTransition> out;
    const std::filesystem::path jsonPath = GuessConnectionsPath(zoneBinPath);
    if (jsonPath.empty()) return out;

    std::ifstream f(jsonPath);
    if (!f) return out;

    const std::string content((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());

    std::size_t pos = 0;
    while ((pos = content.find("\"from_pos\"", pos)) != std::string::npos) {
        const auto from = ParseVec3ArrayAt(content, pos);
        const std::size_t toKey = content.find("\"to_pos\"", pos);
        if (!from || toKey == std::string::npos) {
            pos += 9;
            continue;
        }

        const auto to = ParseVec3ArrayAt(content, toKey);
        if (!to) {
            pos = toKey + 7;
            continue;
        }

        ZoneTransition t{};
        t.from = {from->x, from->y, from->z};
        t.to = {to->x, to->y, to->z};
        t.toZoneName = ParseJsonStringFieldAfter(content, "\"to_zone_name\":", toKey);
        out.push_back(t);
        pos = toKey + 7;
    }

    return out;
}

// ─── 3D visualization modality ─────────────────────────────────────────────────

int RunVizMode(
    const AppArgs& args,
    dtNavMesh* initialNavMesh,
    const std::filesystem::path& zonePath,
    std::vector<Triangle> triangles,
    Vector3 minBounds,
    Vector3 maxBounds)
{
    // ── Zone list ──────────────────────────────────────────────────────────────
    const std::filesystem::path zoneDir =
        args.navMeshDir.empty() ? zonePath.parent_path() : args.navMeshDir;
    auto zones = ScanZoneDir(zoneDir);

    int currentZoneIdx = -1;
    for (int i = 0; i < static_cast<int>(zones.size()); ++i) {
        if (std::filesystem::equivalent(zones[i].path, zonePath)) {
            currentZoneIdx = i;
            break;
        }
    }

    // ── Scene state ────────────────────────────────────────────────────────────
    dtNavMesh* curMesh = initialNavMesh; // we own this
    std::vector<Triangle> navTriangles = std::move(triangles);
    if (!ComputeBounds(navTriangles, &minBounds, &maxBounds)) {
        if (curMesh) dtFreeNavMesh(curMesh);
        return 1;
    }

    auto SceneCenter = [](Vector3 mn, Vector3 mx) -> Vector3 {
        return {0.5f * (mn.x + mx.x), 0.5f * (mn.y + mx.y), 0.5f * (mn.z + mx.z)};
    };
    auto SceneMaxSpan = [](Vector3 mn, Vector3 mx) -> float {
        return std::max({mx.x - mn.x, mx.y - mn.y, mx.z - mn.z, 1.0f});
    };
    auto ResetCamera = [](Camera3D& cam, Vector3 c, float span) {
        cam.position   = {c.x + span * 0.9f, c.y + span * 0.9f, c.z + span * 0.9f};
        cam.target     = c;
        cam.up         = {0.0f, 1.0f, 0.0f};
        cam.fovy       = 60.0f;
        cam.projection = CAMERA_PERSPECTIVE;
    };

    Vector3  center  = SceneCenter(minBounds, maxBounds);
    float    maxSpan = SceneMaxSpan(minBounds, maxBounds);
    float    markerR = std::max(0.3f, maxSpan * 0.004f);

    Camera3D camera{};
    ResetCamera(camera, center, maxSpan);

    // ── Path state ─────────────────────────────────────────────────────────────
    bool  hasStart = args.hasFrom;
    bool  hasEnd   = args.hasTo;
    float startPos[3] = {args.fromPos[0], args.fromPos[1], args.fromPos[2]};
    float endPos[3]   = {args.toPos[0],   args.toPos[1],   args.toPos[2]};
    std::vector<PathWaypoint> path;
    debugnav::ConnectivityIndex connectivity = debugnav::BuildConnectivityIndex(curMesh);
    std::optional<debugnav::PointComponentLabel> startLabel;
    std::optional<debugnav::PointComponentLabel> endLabel;
    std::optional<debugnav::GapDiagnosis> gapDiagnosis;

    if (hasStart && hasEnd) {
        path = ComputePath(curMesh, startPos, endPos);
        startLabel = debugnav::LabelPointOnNavMesh(curMesh, connectivity, startPos);
        endLabel = debugnav::LabelPointOnNavMesh(curMesh, connectivity, endPos);
        gapDiagnosis = debugnav::DiagnosePointGap(curMesh, connectivity, startPos, endPos);
        const std::string zn = zonePath.stem().string();
        PrintPathStdout(path, zn, startPos, endPos);
    } else {
        if (hasStart) startLabel = debugnav::LabelPointOnNavMesh(curMesh, connectivity, startPos);
        if (hasEnd) endLabel = debugnav::LabelPointOnNavMesh(curMesh, connectivity, endPos);
        gapDiagnosis.reset();
    }
    std::vector<ZoneTransition> transitions = LoadZoneTransitions(zonePath);

    // ── Zone-loading helper ────────────────────────────────────────────────────
    auto LoadZone = [&](int idx) -> bool {
        if (idx < 0 || idx >= static_cast<int>(zones.size()) || idx == currentZoneIdx)
            return false;
        dtNavMesh* newMesh = nullptr;
        if (!BuildNavMeshFromFfxiBin(zones[idx].path, &newMesh)) return false;
        auto newNavTris = ExtractTriangles(newMesh);
        if (newNavTris.empty()) { dtFreeNavMesh(newMesh); return false; }
        Vector3 newMin{}, newMax{};
        if (!ComputeBounds(newNavTris, &newMin, &newMax)) { dtFreeNavMesh(newMesh); return false; }

        dtFreeNavMesh(curMesh);
        curMesh        = newMesh;
        navTriangles   = std::move(newNavTris);
        connectivity   = debugnav::BuildConnectivityIndex(curMesh);
        minBounds      = newMin;
        maxBounds      = newMax;
        center         = SceneCenter(minBounds, maxBounds);
        maxSpan        = SceneMaxSpan(minBounds, maxBounds);
        markerR        = std::max(0.3f, maxSpan * 0.004f);
        ResetCamera(camera, center, maxSpan);
        hasStart = hasEnd = false;
        startLabel.reset();
        endLabel.reset();
        gapDiagnosis.reset();
        path.clear();
        transitions    = LoadZoneTransitions(zones[idx].path);
        currentZoneIdx = idx;
        return true;
    };

    // ── UI state ───────────────────────────────────────────────────────────────
    enum class InputMode  { Fly, UI };
    enum class PlaceAction { None, Start, End };

    InputMode   inputMode    = InputMode::Fly;
    PlaceAction placeAction  = PlaceAction::None;
    bool        drawWireframe = false;
    bool        showTransitions = false;
    bool        showAxes = true;
    bool        colorByComponent = false;
    bool        dropdownOpen  = false;
    int         dropScroll    = 0;

    constexpr int kDropW       = 300;
    constexpr int kDropRowH    = 24;
    constexpr int kDropVisible = 18;
    constexpr int kDropHeaderH = 30;

    auto InRect = [](Rectangle r) -> bool {
        return CheckCollisionPointRec(GetMousePosition(), r);
    };

    // ── Window ─────────────────────────────────────────────────────────────────
    SetConfigFlags(FLAG_WINDOW_RESIZABLE | FLAG_MSAA_4X_HINT);
    InitWindow(1600, 900, "FFXI Recast/Detour NavMesh Viewer");
    SetTargetFPS(60);

    while (!WindowShouldClose()) {
        const int sw = GetScreenWidth();
        const int sh = GetScreenHeight();

        // ── Dropdown geometry ──────────────────────────────────────────────────
        const int dropX    = sw - kDropW - 12;
        const int dropY    = 12;
        const int dropListH = std::min(static_cast<int>(zones.size()), kDropVisible) * kDropRowH;

        Rectangle dropHeaderRect{(float)dropX, (float)dropY, (float)kDropW, (float)kDropHeaderH};
        Rectangle dropListRect  {(float)dropX, (float)(dropY + kDropHeaderH), (float)kDropW, (float)dropListH};

        const bool mouseOnDropdown = InRect(dropHeaderRect) ||
                                     (dropdownOpen && InRect(dropListRect));

        // ── Input ──────────────────────────────────────────────────────────────
        if (inputMode == InputMode::Fly) {
            // Use UpdateCameraPro for configurable speed (default free-cam is ~0.09).
            static constexpr float kMoveSpeed  = 0.45f; // 5× faster than Raylib default
            static constexpr float kMouseSens  = 0.1f;  // degrees per pixel (unchanged)
            static constexpr float kScrollSens = 1.5f;
            Vector2 md = GetMouseDelta();
            Vector3 move = {
                (IsKeyDown(KEY_W) ? kMoveSpeed : 0.0f) - (IsKeyDown(KEY_S) ? kMoveSpeed : 0.0f),
                (IsKeyDown(KEY_D) ? kMoveSpeed : 0.0f) - (IsKeyDown(KEY_A) ? kMoveSpeed : 0.0f),
                (IsKeyDown(KEY_E) ? kMoveSpeed : 0.0f) - (IsKeyDown(KEY_Q) ? kMoveSpeed : 0.0f),
            };
            Vector3 rotation = { md.x * kMouseSens, md.y * kMouseSens, 0.0f };
            UpdateCameraPro(&camera, move, rotation, -GetMouseWheelMove() * kScrollSens);
            if (IsKeyPressed(KEY_TAB)) drawWireframe = !drawWireframe;
            if (IsKeyPressed(KEY_B)) showTransitions = !showTransitions;
            if (IsKeyPressed(KEY_G)) showAxes = !showAxes;
            if (IsKeyPressed(KEY_V)) colorByComponent = !colorByComponent;
            if (IsKeyPressed(KEY_F)) {
                inputMode = InputMode::UI;
                EnableCursor();
            }
        } else { // UI mode
            if (IsKeyPressed(KEY_F) || IsKeyPressed(KEY_ESCAPE)) {
                inputMode    = InputMode::UI; // stays UI; next line only exits on Escape
                placeAction  = PlaceAction::None;
                dropdownOpen = false;
                inputMode    = InputMode::Fly;
                // UpdateCamera will re-hide the cursor on the next frame
            }
            if (IsKeyPressed(KEY_S)) placeAction = PlaceAction::Start;
            if (IsKeyPressed(KEY_E)) placeAction = PlaceAction::End;
            if (IsKeyPressed(KEY_C)) {
                path.clear();
                hasStart = hasEnd = false;
                startLabel.reset();
                endLabel.reset();
                gapDiagnosis.reset();
            }
            if (IsKeyPressed(KEY_TAB)) drawWireframe = !drawWireframe;
            if (IsKeyPressed(KEY_B)) showTransitions = !showTransitions;
            if (IsKeyPressed(KEY_G)) showAxes = !showAxes;
            if (IsKeyPressed(KEY_V)) colorByComponent = !colorByComponent;

            // Scroll dropdown list with mouse wheel
            if (dropdownOpen) {
                const float wheel = GetMouseWheelMove();
                if (wheel != 0.0f) {
                    dropScroll = std::clamp(
                        dropScroll - static_cast<int>(wheel),
                        0,
                        std::max(0, static_cast<int>(zones.size()) - kDropVisible));
                }
            }

            // Left-click handling
            if (IsMouseButtonPressed(MOUSE_BUTTON_LEFT)) {
                if (InRect(dropHeaderRect)) {
                    // Toggle dropdown
                    dropdownOpen = !dropdownOpen;
                    placeAction  = PlaceAction::None;
                } else if (dropdownOpen && InRect(dropListRect)) {
                    // Click on a zone row
                    const float ry = GetMousePosition().y - (dropY + kDropHeaderH);
                    const int rowIdx = static_cast<int>(ry / kDropRowH);
                    const int zoneIdx = dropScroll + rowIdx;
                    if (zoneIdx >= 0 && zoneIdx < static_cast<int>(zones.size())) {
                        LoadZone(zoneIdx);
                    }
                    dropdownOpen = false;
                } else if (dropdownOpen && !mouseOnDropdown) {
                    // Click outside dropdown: close it
                    dropdownOpen = false;
                } else if (placeAction != PlaceAction::None && !mouseOnDropdown) {
                    // Raycast against the nav mesh to place a point
                    const Ray ray = GetMouseRay(GetMousePosition(), camera);
                    if (const auto hit = RaycastNavMesh(navTriangles, ray.position, ray.direction)) {
                        const int hitComponentId = (hit->triangleIndex >= 0 && hit->triangleIndex < static_cast<int>(navTriangles.size()))
                            ? navTriangles[hit->triangleIndex].componentId
                            : -1;

                        if (placeAction == PlaceAction::Start) {
                            startPos[0] = hit->position.x; startPos[1] = hit->position.y; startPos[2] = hit->position.z;
                            hasStart    = true;
                            startLabel = debugnav::LabelPointOnNavMesh(curMesh, connectivity, startPos);
                            if (startLabel && startLabel->componentId < 0) {
                                startLabel->componentId = hitComponentId;
                            }
                        } else {
                            endPos[0] = hit->position.x; endPos[1] = hit->position.y; endPos[2] = hit->position.z;
                            hasEnd    = true;
                            endLabel = debugnav::LabelPointOnNavMesh(curMesh, connectivity, endPos);
                            if (endLabel && endLabel->componentId < 0) {
                                endLabel->componentId = hitComponentId;
                            }
                        }
                        placeAction = PlaceAction::None;
                        if (hasStart && hasEnd) {
                            path = ComputePath(curMesh, startPos, endPos);
                            gapDiagnosis = debugnav::DiagnosePointGap(curMesh, connectivity, startPos, endPos);
                            const std::string zn = (currentZoneIdx >= 0)
                                ? zones[currentZoneIdx].name
                                : zonePath.stem().string();
                            PrintPathStdout(path, zn, startPos, endPos);
                        }
                    }
                }
            }
        }

        // ── Drawing ────────────────────────────────────────────────────────────
        BeginDrawing();
        ClearBackground(Color{20, 24, 28, 255});

        rlSetClipPlanes(0.1, static_cast<double>(maxSpan) * 10.0);
        BeginMode3D(camera);

        DrawGrid(40, std::max(1.0f, maxSpan / 40.0f));

        if (showAxes) {
            const float axisLen = std::max(6.0f, maxSpan * 0.12f);
            const Vector3 xEnd{center.x + axisLen, center.y, center.z};
            const Vector3 yEnd{center.x, center.y + axisLen, center.z};
            const Vector3 zEnd{center.x, center.y, center.z + axisLen};
            DrawLine3D(center, xEnd, Color{230, 70, 70, 255});
            DrawLine3D(center, yEnd, Color{70, 220, 110, 255});
            DrawLine3D(center, zEnd, Color{90, 150, 255, 255});
            DrawSphere(xEnd, markerR * 0.75f, Color{230, 70, 70, 255});
            DrawSphere(yEnd, markerR * 0.75f, Color{70, 220, 110, 255});
            DrawSphere(zEnd, markerR * 0.75f, Color{90, 150, 255, 255});
        }

        rlDisableBackfaceCulling();
        if (!drawWireframe) {
            for (const Triangle& tri : navTriangles) {
                const Color c = colorByComponent
                    ? ComponentColorForId(tri.componentId)
                    : Color{80, 190, 120, 255};
                DrawTriangle3D(tri.a, tri.b, tri.c, c);
            }
        } else {
            for (const Triangle& tri : navTriangles) {
                DrawLine3D(tri.a, tri.b, Color{230, 230, 230, 255});
                DrawLine3D(tri.b, tri.c, Color{230, 230, 230, 255});
                DrawLine3D(tri.c, tri.a, Color{230, 230, 230, 255});
            }
        }
        rlEnableBackfaceCulling();

        DrawBoundingBox(BoundingBox{minBounds, maxBounds}, Color{230, 140, 60, 80});

        if (showTransitions) {
            const float h = markerR * 3.0f;
            const float maxLocalLink = std::max(10.0f, maxSpan * 0.25f);
            for (const ZoneTransition& t : transitions) {
                const Vector3 top = {t.from.x, t.from.y + h, t.from.z};
                DrawSphere(t.from, markerR * 0.9f, Color{230, 110, 40, 255});
                DrawLine3D(t.from, top, Color{255, 180, 80, 255});

                const float dx = t.to.x - t.from.x;
                const float dy = t.to.y - t.from.y;
                const float dz = t.to.z - t.from.z;
                const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
                if (dist <= maxLocalLink) {
                    DrawLine3D(t.from, t.to, Color{255, 180, 80, 170});
                    DrawSphere(t.to, markerR * 0.6f, Color{255, 220, 120, 180});
                }
            }
        }

        // Path and endpoint markers
        if (!path.empty()) {
            for (std::size_t i = 0; i + 1 < path.size(); ++i) {
                DrawLine3D({path[i].x, path[i].y, path[i].z},
                           {path[i+1].x, path[i+1].y, path[i+1].z},
                           Color{255, 220, 0, 255});
            }
            for (const auto& wp : path) {
                Color c{255, 220, 0, 255};
                if      (wp.flags & DT_STRAIGHTPATH_START) c = Color{0,   230,  80, 255};
                else if (wp.flags & DT_STRAIGHTPATH_END)   c = Color{230,  60,  60, 255};
                DrawSphere({wp.x, wp.y, wp.z}, markerR, c);
            }
        } else {
            if (hasStart) DrawSphere({startPos[0], startPos[1], startPos[2]}, markerR, Color{0,   230,  80, 200});
            if (hasEnd)   DrawSphere({endPos[0], endPos[1], endPos[2]}, markerR, Color{230,  60,  60, 200});
        }

        EndMode3D();

        // ── HUD: top-left info panel ───────────────────────────────────────────
        const std::string& zoneName = (currentZoneIdx >= 0)
            ? zones[currentZoneIdx].name
            : zonePath.stem().string();

        constexpr int kHudX   = 12;
        constexpr int kLineH  = 22;
        constexpr int kFontSz = 18;
        constexpr int kHudW   = 560;

        int hudLines = 3; // zone + triangles + components
        if (hasStart) ++hudLines;
        if (hasEnd)   ++hudLines;
        if (startLabel && startLabel->onMesh) ++hudLines;
        if (endLabel && endLabel->onMesh) ++hudLines;
        if (!path.empty() || (hasStart && hasEnd)) ++hudLines;
        if (gapDiagnosis.has_value()) {
            hudLines += 2;
            if (!gapDiagnosis->reasons.empty()) ++hudLines;
            if (gapDiagnosis->reasons.size() > 1) ++hudLines;
        }
        hudLines += 2; // controls lines

        int hudY = 12;
        DrawRectangle(kHudX, hudY, kHudW, hudLines * kLineH + 16, Fade(BLACK, 0.55f));
        hudY += 8;

        DrawText(TextFormat("Zone: %s", zoneName.c_str()),
                 kHudX + 8, hudY, kFontSz, RAYWHITE); hudY += kLineH;
        DrawText(TextFormat("Triangles: %d", static_cast<int>(navTriangles.size())),
                 kHudX + 8, hudY, kFontSz, RAYWHITE); hudY += kLineH;
        DrawText(TextFormat("Components: %d  (V: color=%s)",
                            static_cast<int>(connectivity.components.size()),
                            colorByComponent ? "on" : "off"),
                 kHudX + 8, hudY, kFontSz, RAYWHITE); hudY += kLineH;

        if (hasStart) {
            DrawText(TextFormat("From: (%.3f, %.3f, %.3f)", startPos[0], startPos[1], startPos[2]),
                     kHudX + 8, hudY, kFontSz, Color{0, 230, 80, 255}); hudY += kLineH;
        }
        if (startLabel && startLabel->onMesh) {
            DrawText(TextFormat("From component: %d", startLabel->componentId),
                     kHudX + 8, hudY, kFontSz, Color{120, 255, 160, 255}); hudY += kLineH;
        }
        if (hasEnd) {
            DrawText(TextFormat("To:   (%.3f, %.3f, %.3f)", endPos[0], endPos[1], endPos[2]),
                     kHudX + 8, hudY, kFontSz, Color{230, 60, 60, 255}); hudY += kLineH;
        }
        if (endLabel && endLabel->onMesh) {
            DrawText(TextFormat("To component:   %d", endLabel->componentId),
                     kHudX + 8, hudY, kFontSz, Color{255, 150, 150, 255}); hudY += kLineH;
        }
        if (!path.empty()) {
            DrawText(TextFormat("Path waypoints: %d", static_cast<int>(path.size())),
                     kHudX + 8, hudY, kFontSz, Color{255, 220, 0, 255}); hudY += kLineH;
        } else if (hasStart && hasEnd) {
            DrawText("Path: NOT FOUND", kHudX + 8, hudY, kFontSz, Color{255, 80, 80, 255}); hudY += kLineH;
        }

        if (gapDiagnosis.has_value()) {
            const auto& d = *gapDiagnosis;
            DrawText(TextFormat("Diag: same_component=%s sampled=%d min_wall=%.2f",
                                d.sameComponent ? "yes" : "no",
                                d.sampledPoints,
                                d.minWallDistance),
                     kHudX + 8, hudY, kFontSz, Color{255, 220, 140, 255}); hudY += kLineH;
            DrawText(TextFormat("Diag peaks: slope=%.2f deg  step=%.2f",
                                d.maxSlopeDeg,
                                d.maxStepHeight),
                     kHudX + 8, hudY, kFontSz, Color{255, 220, 140, 255}); hudY += kLineH;

            if (!d.reasons.empty()) {
                DrawText(TextFormat("Diag reason 1: %s", d.reasons[0].kind.c_str()),
                         kHudX + 8, hudY, kFontSz, Color{255, 180, 120, 255}); hudY += kLineH;
            }
            if (d.reasons.size() > 1) {
                DrawText(TextFormat("Diag reason 2: %s", d.reasons[1].kind.c_str()),
                         kHudX + 8, hudY, kFontSz, Color{255, 180, 120, 255}); hudY += kLineH;
            }
        }

        if (inputMode == InputMode::Fly) {
            DrawText("WASD+mouse: fly  |  TAB: wireframe  |  B: boundaries  |  G: axes  |  F: UI mode",
                     kHudX + 8, hudY, kFontSz, Color{170, 190, 210, 255}); hudY += kLineH;
        } else {
            DrawText("F/ESC: fly mode  |  TAB: wireframe  |  B: boundaries  |  G: axes  |  C: clear path",
                     kHudX + 8, hudY, kFontSz, Color{170, 190, 210, 255}); hudY += kLineH;
            DrawText("S: place start   |  E: place end   |  V: component colors",
                     kHudX + 8, hudY, kFontSz, Color{170, 190, 210, 255});
        }

        if (showAxes) {
            DrawText("Axes: X red  Y green (up)  Z blue", kHudX + 8, hudY, kFontSz, Color{180, 210, 255, 255});
        }

        if (showTransitions) {
            DrawText(TextFormat("Zone transitions: %d", static_cast<int>(transitions.size())),
                     kHudX + 8, hudY, kFontSz, Color{255, 190, 120, 255});
        }

        // ── HUD: zone dropdown (top-right) ─────────────────────────────────────
        if (!zones.empty()) {
            const bool hdrHovered = InRect(dropHeaderRect) && inputMode == InputMode::UI;
            DrawRectangleRec(dropHeaderRect,
                             hdrHovered ? Color{80, 80, 110, 230} : Color{40, 40, 60, 220});
            DrawRectangleLinesEx(dropHeaderRect, 1, Color{100, 110, 160, 255});

            // Truncate name to fit in the button
            const std::string& dn = (currentZoneIdx >= 0) ? zones[currentZoneIdx].name : zoneName;
            const std::string label = (dn.size() > 30) ? dn.substr(0, 29) + "\xe2\x80\xa6" : dn;
            DrawText(label.c_str(), dropX + 6, dropY + 7, 16, RAYWHITE);
            DrawText(dropdownOpen ? "[^]" : "[v]", dropX + kDropW - 36, dropY + 7, 16, Color{200, 200, 200, 255});

            if (dropdownOpen) {
                DrawRectangleRec(dropListRect, Color{28, 28, 45, 245});
                DrawRectangleLinesEx(dropListRect, 1, Color{100, 110, 160, 255});

                for (int i = 0; i < kDropVisible; ++i) {
                    const int zi = dropScroll + i;
                    if (zi >= static_cast<int>(zones.size())) break;

                    const Rectangle rowR{
                        (float)dropX,
                        (float)(dropY + kDropHeaderH + i * kDropRowH),
                        (float)kDropW,
                        (float)kDropRowH,
                    };
                    const bool rowHov  = InRect(rowR) && inputMode == InputMode::UI;
                    const bool current = (zi == currentZoneIdx);

                    if (current)      DrawRectangleRec(rowR, Color{50, 110, 50, 255});
                    else if (rowHov)  DrawRectangleRec(rowR, Color{60, 60, 100, 255});

                    const std::string& zn2 = zones[zi].name;
                    const std::string  zl  = (zn2.size() > 32) ? zn2.substr(0, 31) + "\xe2\x80\xa6" : zn2;
                    DrawText(zl.c_str(), dropX + 6, dropY + kDropHeaderH + i * kDropRowH + 4, 16, RAYWHITE);
                }

                // Scroll indicator
                if (static_cast<int>(zones.size()) > kDropVisible) {
                    DrawText(TextFormat("%d-%d / %d",
                                        dropScroll + 1,
                                        std::min(dropScroll + kDropVisible, static_cast<int>(zones.size())),
                                        static_cast<int>(zones.size())),
                             dropX + kDropW - 90,
                             dropY + kDropHeaderH + dropListH - kDropRowH + 5,
                             14, Color{160, 160, 160, 255});
                }
            }

            // Mode banner at bottom-center when in UI mode
            if (inputMode == InputMode::UI) {
                const char* banner =
                    (placeAction == PlaceAction::Start) ? "Click mesh to place START  (green)" :
                    (placeAction == PlaceAction::End)   ? "Click mesh to place END    (red)"   :
                                                          "UI mode \xe2\x80\x94 S: start  E: end  C: clear  F/ESC: fly";
                const Color bannerCol =
                    (placeAction == PlaceAction::Start) ? Color{0,   230,  80, 255} :
                    (placeAction == PlaceAction::End)   ? Color{230,  60,  60, 255} :
                                                          RAYWHITE;
                const int tw = MeasureText(banner, 20);
                DrawRectangle(sw / 2 - tw / 2 - 12, sh - 54, tw + 24, 38, Fade(BLACK, 0.7f));
                DrawText(banner, sw / 2 - tw / 2, sh - 46, 20, bannerCol);
            }
        }

        EndDrawing();
    }

    CloseWindow();
    dtFreeNavMesh(curMesh); // we own curMesh (may differ from initialNavMesh after zone switch)
    return 0;
}
