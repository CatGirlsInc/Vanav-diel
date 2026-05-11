#include "world_route.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <unordered_map>
#include <queue>
#include <sstream>

#include <DetourAlloc.h>
#include <DetourNavMesh.h>
#include <DetourNavMeshQuery.h>

extern "C" {
struct sqlite3;
struct sqlite3_stmt;
int sqlite3_open_v2(const char* filename, sqlite3** db, int flags, const char* zVfs);
int sqlite3_close(sqlite3* db);
const char* sqlite3_errmsg(sqlite3* db);
int sqlite3_prepare_v2(sqlite3* db, const char* zSql, int nByte, sqlite3_stmt** ppStmt, const char** pzTail);
int sqlite3_step(sqlite3_stmt* pStmt);
int sqlite3_finalize(sqlite3_stmt* pStmt);
const unsigned char* sqlite3_column_text(sqlite3_stmt* pStmt, int iCol);
double sqlite3_column_double(sqlite3_stmt* pStmt, int iCol);
int sqlite3_column_int(sqlite3_stmt* pStmt, int iCol);
}

constexpr int SQLITE_OK = 0;
constexpr int SQLITE_ROW = 100;
constexpr int SQLITE_OPEN_READONLY = 0x00000001;
constexpr int kMaxSearchNodes = 16384;
constexpr int kMaxPathPolys = 512;
constexpr int kMaxStraightPath = 512;
constexpr float kQueryExtentsCandidates[][3] = {
    {5.0f, 20.0f, 5.0f},
    {15.0f, 120.0f, 15.0f},
    {40.0f, 400.0f, 40.0f},
    {80.0f, 800.0f, 80.0f},
};

struct FfxiNavMeshHeader {
    std::uint32_t magic;
    std::uint32_t version;
    std::uint32_t numTiles;
    float orig[3];
    float tileWidth;
    float tileHeight;
    std::uint32_t maxTiles;
    std::uint32_t maxPolys;
};

struct FfxiNavMeshTileHeader {
    std::uint32_t tileRef;
    std::uint32_t dataSize;
};

struct PathWaypoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    unsigned char flags = 0;
};

constexpr std::uint32_t kFfxiMeshSetMagic = 0x4d534554;
constexpr std::uint32_t kFfxiMeshSetVersion = 1;

namespace {

std::string NormalizeLabel(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (unsigned char ch : text) {
        if (std::isalnum(ch)) {
            out.push_back(static_cast<char>(std::tolower(ch)));
        }
    }
    return out;
}

std::string PrettyLabel(std::string text) {
    std::replace(text.begin(), text.end(), '_', ' ');
    return text;
}

std::string NormalizeZoneStem(const std::string& zoneName) {
    std::string out;
    bool pendingUnderscore = false;
    for (unsigned char ch : zoneName) {
        if (std::isalnum(ch)) {
            if (pendingUnderscore && !out.empty()) {
                out.push_back('_');
            }
            out.push_back(static_cast<char>(std::tolower(ch)));
            pendingUnderscore = false;
        } else {
            pendingUnderscore = !out.empty();
        }
    }
    if (!out.empty() && out.back() == '_') {
        out.pop_back();
    }
    return out;
}

std::filesystem::path GuessZoneMeshPath(const std::filesystem::path& navMeshDir, const std::string& zoneName) {
    return navMeshDir / (NormalizeZoneStem(zoneName) + ".bin");
}

std::string EscapeLuaString(const std::string& text) {
    std::string out;
    out.reserve(text.size());
    for (char ch : text) {
        if (ch == '\\' || ch == '"') {
            out.push_back('\\');
        }
        out.push_back(ch);
    }
    return out;
}

void WriteWalkLegLuaTable(std::ostream& os, const WorldRouteLeg& leg, int indentLevel, bool withReturn) {
    const std::string indent(static_cast<std::size_t>(indentLevel), '\t');
    os << std::fixed;
    if (withReturn) {
        os << indent << "return {\n";
    } else {
        os << indent << "{\n";
    }
    os << indent << "\tmeta = {\n";
    os << indent << "\t\tzone_id = " << leg.zoneId << ",\n";
    os << indent << "\t\tzone_name = \"" << EscapeLuaString(leg.zoneName) << "\",\n";
    os << indent << "\t\tstart_coords = {x = " << leg.startPos[0] << ", y = " << leg.startPos[1] << ", z = " << leg.startPos[2] << "},\n";
    os << indent << "\t\tend_coords = {x = " << leg.endPos[0] << ", y = " << leg.endPos[1] << ", z = " << leg.endPos[2] << "},\n";
    os << indent << "\t},\n";
    os << indent << "\twaypoints = {\n";
    for (const WorldWalkWaypoint& wp : leg.waypoints) {
        os << indent << "\t\t{x = " << wp.x << ", y = " << wp.y << ", z = " << wp.z << "},\n";
    }
    os << indent << "\t},\n";
    os << indent << "}";
}

std::size_t Align4(std::size_t value) {
    return (value + 3u) & ~std::size_t(3u);
}

std::size_t ComputeTileDataSize(const dtMeshHeader& header) {
    std::size_t size = 0;
    size += Align4(sizeof(dtMeshHeader));
    size += Align4(sizeof(float) * 3u * static_cast<std::size_t>(header.vertCount));
    size += Align4(sizeof(dtPoly) * static_cast<std::size_t>(header.polyCount));
    size += Align4(sizeof(dtLink) * static_cast<std::size_t>(header.maxLinkCount));
    size += Align4(sizeof(dtPolyDetail) * static_cast<std::size_t>(header.detailMeshCount));
    size += Align4(sizeof(float) * 3u * static_cast<std::size_t>(header.detailVertCount));
    size += Align4(sizeof(unsigned char) * 4u * static_cast<std::size_t>(header.detailTriCount));
    size += Align4(sizeof(dtBVNode) * static_cast<std::size_t>(header.bvNodeCount));
    size += Align4(sizeof(dtOffMeshConnection) * static_cast<std::size_t>(header.offMeshConCount));
    return size;
}

bool LoadWholeFile(const std::filesystem::path& path, std::vector<unsigned char>* outData) {
    std::ifstream file(path, std::ios::binary | std::ios::ate);
    if (!file) {
        return false;
    }

    const std::streamsize size = file.tellg();
    if (size <= 0) {
        return false;
    }

    outData->resize(static_cast<std::size_t>(size));
    file.seekg(0, std::ios::beg);
    return static_cast<bool>(file.read(reinterpret_cast<char*>(outData->data()), size));
}

bool BuildNavMeshFromFfxiBin(const std::filesystem::path& path, dtNavMesh** outNavMesh) {
    std::vector<unsigned char> data;
    if (!LoadWholeFile(path, &data)) {
        return false;
    }

    if (data.size() < sizeof(FfxiNavMeshHeader)) {
        return false;
    }

    const auto* header = reinterpret_cast<const FfxiNavMeshHeader*>(data.data());
    if (header->magic != kFfxiMeshSetMagic || header->version != kFfxiMeshSetVersion) {
        return false;
    }

    dtNavMeshParams params{};
    params.orig[0] = header->orig[0];
    params.orig[1] = header->orig[1];
    params.orig[2] = header->orig[2];
    params.tileWidth = header->tileWidth;
    params.tileHeight = header->tileHeight;
    params.maxTiles = static_cast<int>(header->maxTiles);
    params.maxPolys = static_cast<int>(header->maxPolys);

    dtNavMesh* navMesh = dtAllocNavMesh();
    if (!navMesh) {
        return false;
    }

    const dtStatus initStatus = navMesh->init(&params);
    if (dtStatusFailed(initStatus)) {
        dtFreeNavMesh(navMesh);
        return false;
    }

    std::size_t offset = sizeof(FfxiNavMeshHeader);
    std::uint32_t loadedTiles = 0;

    while (offset < data.size() && loadedTiles < header->numTiles) {
        if (offset + sizeof(FfxiNavMeshTileHeader) > data.size()) {
            dtFreeNavMesh(navMesh);
            return false;
        }

        const auto* tileHeader = reinterpret_cast<const FfxiNavMeshTileHeader*>(data.data() + offset);
        offset += sizeof(FfxiNavMeshTileHeader);

        if (tileHeader->dataSize == 0 || offset + tileHeader->dataSize > data.size()) {
            dtFreeNavMesh(navMesh);
            return false;
        }

        if (tileHeader->dataSize < sizeof(dtMeshHeader)) {
            dtFreeNavMesh(navMesh);
            return false;
        }

        const auto* meshHeader = reinterpret_cast<const dtMeshHeader*>(data.data() + offset);
        if (meshHeader->magic != DT_NAVMESH_MAGIC || meshHeader->version != DT_NAVMESH_VERSION) {
            dtFreeNavMesh(navMesh);
            return false;
        }

        const std::size_t computedTileDataSize = ComputeTileDataSize(*meshHeader);
        const std::size_t tileDataSize = static_cast<std::size_t>(tileHeader->dataSize);
        if (computedTileDataSize > tileDataSize) {
            dtFreeNavMesh(navMesh);
            return false;
        }

        auto* tileData = static_cast<unsigned char*>(dtAlloc(static_cast<int>(tileDataSize), DT_ALLOC_PERM));
        if (!tileData) {
            dtFreeNavMesh(navMesh);
            return false;
        }

        std::memcpy(tileData, data.data() + offset, tileDataSize);

        const dtStatus addStatus = navMesh->addTile(tileData, static_cast<int>(tileDataSize), DT_TILE_FREE_DATA, 0, nullptr);
        if (dtStatusFailed(addStatus)) {
            dtFree(tileData);
            dtFreeNavMesh(navMesh);
            return false;
        }

        offset += tileDataSize;
        ++loadedTiles;
    }

    *outNavMesh = navMesh;
    return true;
}

std::vector<PathWaypoint> ComputePath(
    const dtNavMesh* navMesh,
    const float* startPos,
    const float* endPos)
{
    for (const float (&halfExtents)[3] : kQueryExtentsCandidates) {
        dtNavMeshQuery* query = dtAllocNavMeshQuery();
        if (!query) {
            return {};
        }

        dtStatus status = query->init(navMesh, kMaxSearchNodes);
        if (dtStatusFailed(status)) {
            dtFreeNavMeshQuery(query);
            return {};
        }

        dtQueryFilter filter;

        dtPolyRef startRef = 0;
        dtPolyRef endRef = 0;
        float snappedStart[3];
        float snappedEnd[3];

        status = query->findNearestPoly(startPos, halfExtents, &filter, &startRef, snappedStart);
        if (dtStatusFailed(status) || startRef == 0) {
            dtFreeNavMeshQuery(query);
            continue;
        }

        status = query->findNearestPoly(endPos, halfExtents, &filter, &endRef, snappedEnd);
        if (dtStatusFailed(status) || endRef == 0) {
            dtFreeNavMeshQuery(query);
            continue;
        }

        dtPolyRef polys[kMaxPathPolys];
        int polyCount = 0;
        status = query->findPath(startRef, endRef, snappedStart, snappedEnd, &filter, polys, &polyCount, kMaxPathPolys);
        if (dtStatusFailed(status) || polyCount == 0) {
            dtFreeNavMeshQuery(query);
            continue;
        }

        float straightPath[kMaxStraightPath * 3];
        unsigned char straightPathFlags[kMaxStraightPath];
        dtPolyRef straightPathRefs[kMaxStraightPath];
        int straightPathCount = 0;

        status = query->findStraightPath(
            snappedStart, snappedEnd,
            polys, polyCount,
            straightPath, straightPathFlags, straightPathRefs,
            &straightPathCount, kMaxStraightPath);

        if (dtStatusFailed(status) || straightPathCount == 0) {
            dtFreeNavMeshQuery(query);
            continue;
        }

        std::vector<PathWaypoint> result;
        result.reserve(static_cast<std::size_t>(straightPathCount));
        for (int i = 0; i < straightPathCount; ++i) {
            result.push_back({
                straightPath[i * 3 + 0],
                straightPath[i * 3 + 1],
                straightPath[i * 3 + 2],
                straightPathFlags[i],
            });
        }

        dtFreeNavMeshQuery(query);
        return result;
    }

    return {};
}

class ZoneNavMeshCache {
public:
    ~ZoneNavMeshCache() {
        for (auto& item : cache_) {
            dtFreeNavMesh(item.second);
        }
    }

    dtNavMesh* Get(const std::filesystem::path& navMeshDir, const std::string& zoneName, std::vector<std::string>* warnings) {
        const std::string key = NormalizeZoneStem(zoneName);
        const auto it = cache_.find(key);
        if (it != cache_.end()) {
            return it->second;
        }

        dtNavMesh* navMesh = nullptr;
        const std::filesystem::path path = GuessZoneMeshPath(navMeshDir, zoneName);
        if (!BuildNavMeshFromFfxiBin(path, &navMesh)) {
            if (warnings) warnings->push_back("failed to load navmesh: " + path.string());
            return nullptr;
        }

        cache_[key] = navMesh;
        return navMesh;
    }

private:
    std::unordered_map<std::string, dtNavMesh*> cache_;
};

bool ComputeWalkLeg(
    ZoneNavMeshCache* cache,
    const std::filesystem::path& navMeshDir,
    const std::string& zoneName,
    const float startPos[3],
    const float endPos[3],
    std::vector<WorldWalkWaypoint>* outWaypoints,
    std::vector<std::string>* warnings)
{
    const dtNavMesh* navMesh = cache->Get(navMeshDir, zoneName, warnings);
    if (!navMesh) {
        return false;
    }

    const std::vector<PathWaypoint> path = ComputePath(navMesh, startPos, endPos);
    if (path.empty()) {
        if (warnings) warnings->push_back("no intra-zone path found in " + zoneName);
        return false;
    }

    outWaypoints->clear();
    outWaypoints->reserve(path.size());
    for (const PathWaypoint& wp : path) {
        outWaypoints->push_back({wp.x, wp.y, wp.z, wp.flags});
    }
    return true;
}

void AppendWalkLeg(
    WorldRoute* route,
    ZoneNavMeshCache* cache,
    int zoneId,
    const std::filesystem::path& navMeshDir,
    const std::string& zoneName,
    const float startPos[3],
    const float endPos[3],
    const std::string& fromLabel,
    const std::string& toLabel,
    float cost,
    std::vector<std::string>* warnings)
{
    std::vector<WorldWalkWaypoint> waypoints;
    if (!ComputeWalkLeg(cache, navMeshDir, zoneName, startPos, endPos, &waypoints, warnings)) {
        return;
    }

    WorldRouteLeg leg;
    leg.kind = WorldRouteLeg::Kind::Walk;
    leg.zoneId = zoneId;
    leg.zoneName = zoneName;
    leg.fromLabel = fromLabel;
    leg.toLabel = toLabel;
    leg.waypoints = std::move(waypoints);
    leg.startPos[0] = startPos[0];
    leg.startPos[1] = startPos[1];
    leg.startPos[2] = startPos[2];
    leg.endPos[0] = endPos[0];
    leg.endPos[1] = endPos[1];
    leg.endPos[2] = endPos[2];
    leg.cost = cost;
    route->legs.push_back(std::move(leg));
}

void AppendWarpLeg(WorldRoute* route, const WorldRouteStep& step) {
    WorldRouteLeg leg;
    leg.kind = WorldRouteLeg::Kind::Warp;
    leg.zoneName = step.fromZoneName;
    leg.label = PrettyLabel(step.connection.type);
    leg.fromLabel = step.fromZoneName;
    leg.toLabel = step.toZoneName;
    leg.connection = step.connection;
    leg.cost = step.connection.cost;
    route->legs.push_back(std::move(leg));
}

struct HomePointAnchor {
    int number = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
};

using HomePointIndex = std::unordered_map<int, std::vector<HomePointAnchor>>;

std::optional<int> ParseHomePointNumber(const std::string& text) {
    std::size_t hashPos = text.find('#');
    if (hashPos == std::string::npos || hashPos + 1 >= text.size()) {
        return std::nullopt;
    }

    int value = 0;
    bool hasDigit = false;
    for (std::size_t i = hashPos + 1; i < text.size(); ++i) {
        const unsigned char ch = static_cast<unsigned char>(text[i]);
        if (!std::isdigit(ch)) {
            break;
        }
        hasDigit = true;
        value = value * 10 + static_cast<int>(ch - '0');
    }

    if (!hasDigit) {
        return std::nullopt;
    }
    return value;
}

HomePointIndex BuildHomePointIndex(const WorldGraph& graph) {
    HomePointIndex index;
    for (const WorldNpc& npc : graph.npcs) {
        std::optional<int> number = ParseHomePointNumber(npc.name);
        if (!number) {
            number = ParseHomePointNumber(npc.polutilsName);
        }
        if (!number) {
            continue;
        }

        index[npc.zoneId].push_back(HomePointAnchor{*number, npc.posX, npc.posY, npc.posZ});
    }
    return index;
}

std::optional<int> FindNearestHomePointNumber(
    const HomePointIndex& index,
    int zoneId,
    float x,
    float y,
    float z)
{
    const auto it = index.find(zoneId);
    if (it == index.end() || it->second.empty()) {
        return std::nullopt;
    }

    float bestDist = std::numeric_limits<float>::infinity();
    int bestNumber = 0;
    for (const HomePointAnchor& hp : it->second) {
        const float dx = x - hp.x;
        const float dy = y - hp.y;
        const float dz = z - hp.z;
        const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        if (dist < bestDist) {
            bestDist = dist;
            bestNumber = hp.number;
        }
    }

    if (!std::isfinite(bestDist)) {
        return std::nullopt;
    }
    return bestNumber;
}

std::string FormatHomePointSuffix(const std::optional<int>& number) {
    if (!number) {
        return {};
    }
    return " (HP#" + std::to_string(*number) + ")";
}

std::string FormatHomePointHop(const std::optional<int>& fromNumber, const std::optional<int>& toNumber) {
    if (!fromNumber && !toNumber) {
        return {};
    }
    if (fromNumber && toNumber) {
        return "  hp=" + std::to_string(*fromNumber) + "->" + std::to_string(*toNumber);
    }
    if (fromNumber) {
        return "  hp=" + std::to_string(*fromNumber) + "->?";
    }
    return "  hp=?->" + std::to_string(*toNumber);
}

bool IsMultiAnchorWarpType(const std::string& type) {
    return type == "homepoint" || type == "waypoint";
}

float Distance3(const float ax, const float ay, const float az,
                const float bx, const float by, const float bz) {
    const float dx = ax - bx;
    const float dy = ay - by;
    const float dz = az - bz;
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

std::vector<WorldConnection> BuildStepCandidates(
    const WorldGraph& graph,
    const WorldRouteStep& step,
    std::vector<std::string>* outWarnings)
{
    std::vector<WorldConnection> candidates;

    if (!IsMultiAnchorWarpType(step.connection.type)) {
        candidates.push_back(step.connection);
        return candidates;
    }

    const auto outIt = graph.outgoingByZoneId.find(step.connection.fromZoneId);
    if (outIt == graph.outgoingByZoneId.end()) {
        candidates.push_back(step.connection);
        return candidates;
    }

    constexpr float kCostEpsilon = 0.0001f;
    for (const std::size_t edgeIndex : outIt->second) {
        const WorldConnection& edge = graph.connections[edgeIndex];
        if (edge.type != step.connection.type) {
            continue;
        }
        if (edge.toZoneId != step.connection.toZoneId) {
            continue;
        }
        if (std::fabs(edge.cost - step.connection.cost) > kCostEpsilon) {
            continue;
        }
        candidates.push_back(edge);
    }

    if (candidates.empty()) {
        if (outWarnings) {
            outWarnings->push_back(
                "no alternative anchors found for " + step.connection.type +
                " in " + step.fromZoneName + " -> " + step.toZoneName + "; using route edge");
        }
        candidates.push_back(step.connection);
    }

    return candidates;
}

std::vector<WorldConnection> SelectBestAnchorChain(
    const WorldRoute& route,
    const WorldGraph& graph,
    std::vector<std::string>* outWarnings)
{
    if (route.steps.empty()) {
        return {};
    }

    std::vector<std::vector<WorldConnection>> candidates;
    candidates.reserve(route.steps.size());
    for (const WorldRouteStep& step : route.steps) {
        candidates.push_back(BuildStepCandidates(graph, step, outWarnings));
    }

    std::vector<std::vector<float>> dp(route.steps.size());
    std::vector<std::vector<int>> parent(route.steps.size());
    for (std::size_t i = 0; i < route.steps.size(); ++i) {
        dp[i].assign(candidates[i].size(), std::numeric_limits<float>::infinity());
        parent[i].assign(candidates[i].size(), -1);
    }

    for (std::size_t j = 0; j < candidates[0].size(); ++j) {
        const WorldConnection& c = candidates[0][j];
        dp[0][j] = Distance3(
            route.start.npc.posX, route.start.npc.posY, route.start.npc.posZ,
            c.fromX, c.fromY, c.fromZ);
    }

    for (std::size_t i = 1; i < route.steps.size(); ++i) {
        for (std::size_t j = 0; j < candidates[i].size(); ++j) {
            const WorldConnection& cur = candidates[i][j];
            for (std::size_t k = 0; k < candidates[i - 1].size(); ++k) {
                const WorldConnection& prev = candidates[i - 1][k];
                const float transition = Distance3(prev.toX, prev.toY, prev.toZ, cur.fromX, cur.fromY, cur.fromZ);
                const float score = dp[i - 1][k] + transition;
                if (score < dp[i][j]) {
                    dp[i][j] = score;
                    parent[i][j] = static_cast<int>(k);
                }
            }
        }
    }

    const std::size_t last = route.steps.size() - 1;
    int bestLast = -1;
    float bestScore = std::numeric_limits<float>::infinity();
    for (std::size_t j = 0; j < candidates[last].size(); ++j) {
        const WorldConnection& c = candidates[last][j];
        const float tail = Distance3(c.toX, c.toY, c.toZ, route.end.npc.posX, route.end.npc.posY, route.end.npc.posZ);
        const float score = dp[last][j] + tail;
        if (score < bestScore) {
            bestScore = score;
            bestLast = static_cast<int>(j);
        }
    }

    if (bestLast < 0) {
        if (outWarnings) {
            outWarnings->push_back("anchor optimization failed; falling back to route edges");
        }
        std::vector<WorldConnection> fallback;
        fallback.reserve(route.steps.size());
        for (const WorldRouteStep& step : route.steps) {
            fallback.push_back(step.connection);
        }
        return fallback;
    }

    std::vector<WorldConnection> selected(route.steps.size());
    int current = bestLast;
    for (std::size_t idx = route.steps.size(); idx-- > 0;) {
        selected[idx] = candidates[idx][static_cast<std::size_t>(current)];
        if (idx > 0) {
            current = parent[idx][static_cast<std::size_t>(current)];
            if (current < 0) {
                if (outWarnings) {
                    outWarnings->push_back("anchor optimization chain reconstruction failed; falling back to route edges");
                }
                selected.clear();
                for (const WorldRouteStep& step : route.steps) {
                    selected.push_back(step.connection);
                }
                return selected;
            }
        }
    }

    return selected;
}

int LevenshteinDistance(const std::string& a, const std::string& b) {
    std::vector<int> prev(b.size() + 1), cur(b.size() + 1);
    for (std::size_t j = 0; j < prev.size(); ++j) {
        prev[j] = static_cast<int>(j);
    }
    for (std::size_t i = 1; i <= a.size(); ++i) {
        cur[0] = static_cast<int>(i);
        for (std::size_t j = 1; j <= b.size(); ++j) {
            const int cost = (a[i - 1] == b[j - 1]) ? 0 : 1;
            cur[j] = std::min({
                prev[j] + 1,
                cur[j - 1] + 1,
                prev[j - 1] + cost,
            });
        }
        prev.swap(cur);
    }
    return prev.back();
}

std::string ReadText(sqlite3_stmt* stmt, int column) {
    const unsigned char* text = sqlite3_column_text(stmt, column);
    return text ? reinterpret_cast<const char*>(text) : std::string{};
}

bool LoadZones(sqlite3* db, WorldGraph* graph, std::string* outError) {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, "SELECT zone_id, name, canon FROM zones ORDER BY zone_id;", -1, &stmt, nullptr) != SQLITE_OK) {
        if (outError) *outError = sqlite3_errmsg(db);
        return false;
    }

    while (sqlite3_step(stmt) == SQLITE_ROW) {
        WorldZone zone;
        zone.zoneId = sqlite3_column_int(stmt, 0);
        zone.name = ReadText(stmt, 1);
        zone.canon = ReadText(stmt, 2);
        graph->zoneIdsByNormalizedName[NormalizeLabel(zone.name)] = zone.zoneId;
        graph->zoneIdsByNormalizedName[NormalizeLabel(zone.canon)] = zone.zoneId;
        graph->zonesById[zone.zoneId] = std::move(zone);
    }

    sqlite3_finalize(stmt);
    return true;
}

bool LoadNpcs(sqlite3* db, WorldGraph* graph, std::string* outError) {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, "SELECT npcid, name, polutils_name, zone_id, zone_name, pos_x, pos_y, pos_z, pos_rot, content_tag FROM npcs ORDER BY npcid;", -1, &stmt, nullptr) != SQLITE_OK) {
        if (outError) *outError = sqlite3_errmsg(db);
        return false;
    }

    std::size_t index = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        WorldNpc npc;
        npc.npcid = sqlite3_column_int(stmt, 0);
        npc.name = ReadText(stmt, 1);
        npc.polutilsName = ReadText(stmt, 2);
        npc.zoneId = sqlite3_column_int(stmt, 3);
        npc.zoneName = ReadText(stmt, 4);
        npc.posX = static_cast<float>(sqlite3_column_double(stmt, 5));
        npc.posY = static_cast<float>(sqlite3_column_double(stmt, 6));
        npc.posZ = static_cast<float>(sqlite3_column_double(stmt, 7));
        npc.posRot = sqlite3_column_int(stmt, 8);
        npc.contentTag = ReadText(stmt, 9);

        graph->npcs.push_back(std::move(npc));
        ++index;
    }

    sqlite3_finalize(stmt);
    return true;
}

bool LoadConnections(sqlite3* db, WorldGraph* graph, std::string* outError) {
    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db, "SELECT id, type, from_zone, from_x, from_y, from_z, from_rot, to_zone, to_x, to_y, to_z, to_rot, cost, requires, meta FROM connections ORDER BY id;", -1, &stmt, nullptr) != SQLITE_OK) {
        if (outError) *outError = sqlite3_errmsg(db);
        return false;
    }

    std::size_t index = 0;
    while (sqlite3_step(stmt) == SQLITE_ROW) {
        WorldConnection connection;
        connection.id = sqlite3_column_int(stmt, 0);
        connection.type = ReadText(stmt, 1);
        connection.fromZoneId = sqlite3_column_int(stmt, 2);
        connection.fromX = static_cast<float>(sqlite3_column_double(stmt, 3));
        connection.fromY = static_cast<float>(sqlite3_column_double(stmt, 4));
        connection.fromZ = static_cast<float>(sqlite3_column_double(stmt, 5));
        connection.fromRot = static_cast<float>(sqlite3_column_double(stmt, 6));
        connection.toZoneId = sqlite3_column_int(stmt, 7);
        connection.toX = static_cast<float>(sqlite3_column_double(stmt, 8));
        connection.toY = static_cast<float>(sqlite3_column_double(stmt, 9));
        connection.toZ = static_cast<float>(sqlite3_column_double(stmt, 10));
        connection.toRot = static_cast<float>(sqlite3_column_double(stmt, 11));
        connection.cost = static_cast<float>(sqlite3_column_double(stmt, 12));
        connection.requiresText = ReadText(stmt, 13);
        connection.meta = ReadText(stmt, 14);
        if (connection.type == "survival_guide") {
            // Survival guide groups are fully connected and not useful for route decisions.
            connection.meta.clear();
        }

        graph->outgoingByZoneId[connection.fromZoneId].push_back(index);
        graph->connections.push_back(std::move(connection));
        ++index;
    }

    sqlite3_finalize(stmt);
    return true;
}

} // namespace

bool LoadWorldGraph(const std::filesystem::path& dbPath, WorldGraph* outGraph, std::string* outError) {
    if (!outGraph) {
        if (outError) *outError = "outGraph is null";
        return false;
    }

    sqlite3* db = nullptr;
    if (sqlite3_open_v2(dbPath.string().c_str(), &db, SQLITE_OPEN_READONLY, nullptr) != SQLITE_OK) {
        if (outError) *outError = db ? sqlite3_errmsg(db) : "failed to open database";
        if (db) sqlite3_close(db);
        return false;
    }

    const bool ok = LoadZones(db, outGraph, outError) && LoadNpcs(db, outGraph, outError) && LoadConnections(db, outGraph, outError);
    sqlite3_close(db);
    return ok;
}

std::optional<WorldTarget> ResolveWorldTarget(
    const WorldGraph& graph,
    const std::string& query,
    std::vector<std::string>* outWarnings)
{
    const std::string normalizedQuery = NormalizeLabel(query);
    if (normalizedQuery.empty()) {
        if (outWarnings) outWarnings->push_back("empty target query");
        return std::nullopt;
    }

    int bestScore = std::numeric_limits<int>::min();
    std::size_t bestIndex = 0;
    std::string bestLabel;
    std::string bestKind;

    for (std::size_t index = 0; index < graph.npcs.size(); ++index) {
        const WorldNpc& npc = graph.npcs[index];
        const std::vector<std::string> labels = {
            npc.name,
            npc.polutilsName,
            npc.zoneName + " " + npc.name,
            npc.zoneName + " " + npc.polutilsName,
        };

        for (const std::string& label : labels) {
            if (label.empty()) {
                continue;
            }

            const std::string normalizedLabel = NormalizeLabel(label);
            int score = 0;
            std::string kind;
            if (normalizedQuery == normalizedLabel) {
                score = 10000;
                kind = "exact";
            } else if (normalizedLabel.find(normalizedQuery) != std::string::npos || normalizedQuery.find(normalizedLabel) != std::string::npos) {
                score = 9000;
                kind = "substring";
            } else {
                const int distance = LevenshteinDistance(normalizedQuery, normalizedLabel);
                const int longest = static_cast<int>(std::max(normalizedQuery.size(), normalizedLabel.size()));
                score = 5000 - distance * 100 - longest;
                kind = "fuzzy";
            }

            if (score > bestScore) {
                bestScore = score;
                bestIndex = index;
                bestLabel = label;
                bestKind = kind;
            }
        }
    }

    if (bestScore < 0) {
        if (outWarnings) outWarnings->push_back("no world target matched query: " + query);
        return std::nullopt;
    }

    if (bestKind == "fuzzy" && outWarnings) {
        outWarnings->push_back("fuzzy target match used for: " + query + " -> " + PrettyLabel(bestLabel));
    }

    WorldTarget target;
    target.npc = graph.npcs[bestIndex];
    target.matchedLabel = PrettyLabel(bestLabel);
    target.matchKind = bestKind;
    target.matchScore = bestScore;
    return target;
}

std::optional<WorldRoute> FindWorldRoute(
    const WorldGraph& graph,
    const WorldTarget& start,
    const WorldTarget& end,
    std::vector<std::string>* outWarnings)
{
    if (start.npc.zoneId == end.npc.zoneId) {
        WorldRoute route;
        route.start = start;
        route.end = end;
        return route;
    }

    struct QueueItem {
        int zoneId;
        float cost;
        bool operator>(const QueueItem& other) const { return cost > other.cost; }
    };

    std::priority_queue<QueueItem, std::vector<QueueItem>, std::greater<QueueItem>> queue;
    std::unordered_map<int, float> bestCost;
    std::unordered_map<int, std::pair<int, std::size_t>> previous;

    queue.push({start.npc.zoneId, 0.0f});
    bestCost[start.npc.zoneId] = 0.0f;

    while (!queue.empty()) {
        const QueueItem current = queue.top();
        queue.pop();

        const auto bestIt = bestCost.find(current.zoneId);
        if (bestIt == bestCost.end() || current.cost > bestIt->second) {
            continue;
        }
        if (current.zoneId == end.npc.zoneId) {
            break;
        }

        const auto outgoingIt = graph.outgoingByZoneId.find(current.zoneId);
        if (outgoingIt == graph.outgoingByZoneId.end()) {
            continue;
        }

        for (std::size_t edgeIndex : outgoingIt->second) {
            const WorldConnection& connection = graph.connections[edgeIndex];
            const float nextCost = current.cost + std::max(0.01f, connection.cost);
            const auto nextBestIt = bestCost.find(connection.toZoneId);
            if (nextBestIt != bestCost.end() && nextCost >= nextBestIt->second) {
                continue;
            }

            bestCost[connection.toZoneId] = nextCost;
            previous[connection.toZoneId] = {current.zoneId, edgeIndex};
            queue.push({connection.toZoneId, nextCost});
        }
    }

    if (bestCost.find(end.npc.zoneId) == bestCost.end()) {
        if (outWarnings) outWarnings->push_back("no zone route found from " + start.npc.zoneName + " to " + end.npc.zoneName);
        return std::nullopt;
    }

    std::vector<WorldRouteStep> reversedSteps;
    int zoneId = end.npc.zoneId;
    while (zoneId != start.npc.zoneId) {
        const auto prevIt = previous.find(zoneId);
        if (prevIt == previous.end()) {
            if (outWarnings) outWarnings->push_back("route reconstruction failed");
            return std::nullopt;
        }

        const int prevZone = prevIt->second.first;
        const std::size_t edgeIndex = prevIt->second.second;
        WorldRouteStep step;
        step.connection = graph.connections[edgeIndex];
        step.fromZoneName = graph.zonesById.count(prevZone) ? graph.zonesById.at(prevZone).name : std::to_string(prevZone);
        step.toZoneName = graph.zonesById.count(zoneId) ? graph.zonesById.at(zoneId).name : std::to_string(zoneId);
        reversedSteps.push_back(std::move(step));
        zoneId = prevZone;
    }

    std::reverse(reversedSteps.begin(), reversedSteps.end());

    WorldRoute route;
    route.start = start;
    route.end = end;
    route.steps = std::move(reversedSteps);
    route.totalCost = bestCost[end.npc.zoneId];
    return route;
}

void PrintWorldRoute(
    const WorldRoute& route,
    const WorldGraph& graph,
    const std::filesystem::path& navMeshDir,
    bool fileOutput,
    std::vector<std::string>* outWarnings)
{
    ZoneNavMeshCache cache;
    WorldRoute expanded = route;
    const HomePointIndex homePointIndex = BuildHomePointIndex(graph);
    const std::vector<WorldConnection> selectedConnections = SelectBestAnchorChain(expanded, graph, outWarnings);

    int currentZoneId = expanded.start.npc.zoneId;
    float currentPos[3] = {expanded.start.npc.posX, expanded.start.npc.posY, expanded.start.npc.posZ};
    std::string currentLabel = expanded.start.matchedLabel;

    for (std::size_t stepIndex = 0; stepIndex < expanded.steps.size(); ++stepIndex) {
        WorldRouteStep step = expanded.steps[stepIndex];
        if (stepIndex < selectedConnections.size()) {
            step.connection = selectedConnections[stepIndex];
            expanded.steps[stepIndex].connection = selectedConnections[stepIndex];
        }

        const auto fromZoneIt = graph.zonesById.find(step.connection.fromZoneId);
        const auto toZoneIt = graph.zonesById.find(step.connection.toZoneId);
        if (fromZoneIt == graph.zonesById.end() || toZoneIt == graph.zonesById.end()) {
            if (outWarnings) outWarnings->push_back("route references an unknown zone id");
            continue;
        }

        if (currentZoneId != step.connection.fromZoneId) {
            if (outWarnings) outWarnings->push_back("route expansion encountered a zone mismatch before " + fromZoneIt->second.name);
            continue;
        }

        std::optional<int> fromHomePointNumber;
        std::optional<int> toHomePointNumber;
        if (step.connection.type == "homepoint") {
            fromHomePointNumber = FindNearestHomePointNumber(
                homePointIndex,
                step.connection.fromZoneId,
                step.connection.fromX,
                step.connection.fromY,
                step.connection.fromZ);
            toHomePointNumber = FindNearestHomePointNumber(
                homePointIndex,
                step.connection.toZoneId,
                step.connection.toX,
                step.connection.toY,
                step.connection.toZ);
        }

        const float anchorPos[3] = {step.connection.fromX, step.connection.fromY, step.connection.fromZ};
        const std::string walkToLabel = PrettyLabel(step.connection.type) + FormatHomePointSuffix(fromHomePointNumber)
            + " @ " + fromZoneIt->second.name;
        AppendWalkLeg(
            &expanded,
            &cache,
            step.connection.fromZoneId,
            navMeshDir,
            fromZoneIt->second.name,
            currentPos,
            anchorPos,
            currentLabel,
            walkToLabel,
            0.0f,
            outWarnings);

        step.fromZoneName += FormatHomePointSuffix(fromHomePointNumber);
        step.toZoneName += FormatHomePointSuffix(toHomePointNumber);
        AppendWarpLeg(&expanded, step);

        currentZoneId = step.connection.toZoneId;
        currentPos[0] = step.connection.toX;
        currentPos[1] = step.connection.toY;
        currentPos[2] = step.connection.toZ;
        currentLabel = PrettyLabel(step.connection.type) + FormatHomePointSuffix(toHomePointNumber)
            + " arrival @ " + toZoneIt->second.name;
    }

    const auto endZoneIt = graph.zonesById.find(expanded.end.npc.zoneId);
    if (endZoneIt != graph.zonesById.end() && currentZoneId == expanded.end.npc.zoneId) {
        const float endPos[3] = {expanded.end.npc.posX, expanded.end.npc.posY, expanded.end.npc.posZ};
        AppendWalkLeg(
            &expanded,
            &cache,
            expanded.end.npc.zoneId,
            navMeshDir,
            endZoneIt->second.name,
            currentPos,
            endPos,
            currentLabel,
            expanded.end.matchedLabel,
            0.0f,
            outWarnings);
    }

    std::cout << std::fixed;
    std::cout << "World route\n";
    std::cout << "Start: " << expanded.start.npc.zoneName << " / " << expanded.start.matchedLabel << "\n";
    std::cout << "End:   " << expanded.end.npc.zoneName << " / " << expanded.end.matchedLabel << "\n";
    std::cout << "Match: " << expanded.start.matchKind << " -> " << expanded.end.matchKind << '\n';
    std::cout << "Total cost: " << expanded.totalCost << '\n';
    std::cout << "Steps: " << expanded.steps.size() << '\n';
    std::cout << "---\n";
    for (std::size_t index = 0; index < expanded.legs.size(); ++index) {
        const WorldRouteLeg& leg = expanded.legs[index];
        if (leg.kind == WorldRouteLeg::Kind::Walk) {
            std::cout << index << "  walk  " << leg.zoneName << "  " << leg.fromLabel << " -> " << leg.toLabel
                      << "  waypoints=" << leg.waypoints.size() << '\n';
            if (fileOutput) {
                WriteWalkLegLuaTable(std::cout, leg, 1, true);
                std::cout << '\n';
            } else {
                for (std::size_t wpIndex = 0; wpIndex < leg.waypoints.size(); ++wpIndex) {
                    const WorldWalkWaypoint& wp = leg.waypoints[wpIndex];
                    std::cout << "     " << wpIndex << "  " << wp.x << ", " << wp.y << ", " << wp.z;
                    if (wp.flags != 0) {
                        std::cout << "  flags=" << static_cast<int>(wp.flags);
                    }
                    std::cout << '\n';
                }
            }
        } else {
            std::string homePointHop;
            if (leg.connection.type == "homepoint") {
                const std::optional<int> fromHomePointNumber = FindNearestHomePointNumber(
                    homePointIndex,
                    leg.connection.fromZoneId,
                    leg.connection.fromX,
                    leg.connection.fromY,
                    leg.connection.fromZ);
                const std::optional<int> toHomePointNumber = FindNearestHomePointNumber(
                    homePointIndex,
                    leg.connection.toZoneId,
                    leg.connection.toX,
                    leg.connection.toY,
                    leg.connection.toZ);
                homePointHop = FormatHomePointHop(fromHomePointNumber, toHomePointNumber);
            }

            std::cout << index << "  warp  " << leg.fromLabel << " -> " << leg.toLabel
                      << "  type=" << leg.connection.type << "  cost=" << leg.connection.cost;
            if (!homePointHop.empty()) {
                std::cout << homePointHop;
            }
            if (!leg.connection.requiresText.empty()) {
                std::cout << "  requires=" << leg.connection.requiresText;
            }
            if (!leg.connection.meta.empty()) {
                std::cout << "  meta=" << leg.connection.meta;
            }
            std::cout << '\n';
        }
    }
    if (!expanded.steps.empty()) {
        std::cout << "Zone itinerary\n";
        for (std::size_t index = 0; index < expanded.steps.size(); ++index) {
            const WorldRouteStep& step = expanded.steps[index];
            std::string homePointHop;
            if (step.connection.type == "homepoint") {
                const std::optional<int> fromHomePointNumber = FindNearestHomePointNumber(
                    homePointIndex,
                    step.connection.fromZoneId,
                    step.connection.fromX,
                    step.connection.fromY,
                    step.connection.fromZ);
                const std::optional<int> toHomePointNumber = FindNearestHomePointNumber(
                    homePointIndex,
                    step.connection.toZoneId,
                    step.connection.toX,
                    step.connection.toY,
                    step.connection.toZ);
                homePointHop = FormatHomePointHop(fromHomePointNumber, toHomePointNumber);
            }

            std::cout << index << "  " << step.connection.type << "  " << step.fromZoneName << " -> " << step.toZoneName
                      << "  cost=" << step.connection.cost;
            if (!homePointHop.empty()) {
                std::cout << homePointHop;
            }
            if (!step.connection.requiresText.empty()) {
                std::cout << "  requires=" << step.connection.requiresText;
            }
            if (!step.connection.meta.empty()) {
                std::cout << "  meta=" << step.connection.meta;
            }
            std::cout << '\n';
        }
    }
    std::cout << "---\n";

    std::ofstream zoneFile("route_zones.txt");
    if (zoneFile) {
        zoneFile << std::fixed;
        zoneFile << "Zone Itinerary\n";
        zoneFile << "Start: " << expanded.start.npc.zoneName << " / " << expanded.start.matchedLabel << "\n";
        zoneFile << "End:   " << expanded.end.npc.zoneName << " / " << expanded.end.matchedLabel << "\n";
        zoneFile << "Total cost: " << expanded.totalCost << "\n";
        zoneFile << "---\n";
        for (std::size_t index = 0; index < expanded.steps.size(); ++index) {
            const WorldRouteStep& step = expanded.steps[index];
            std::string homePointHop;
            if (step.connection.type == "homepoint") {
                const std::optional<int> fromHomePointNumber = FindNearestHomePointNumber(
                    homePointIndex,
                    step.connection.fromZoneId,
                    step.connection.fromX,
                    step.connection.fromY,
                    step.connection.fromZ);
                const std::optional<int> toHomePointNumber = FindNearestHomePointNumber(
                    homePointIndex,
                    step.connection.toZoneId,
                    step.connection.toX,
                    step.connection.toY,
                    step.connection.toZ);
                homePointHop = FormatHomePointHop(fromHomePointNumber, toHomePointNumber);
            }

            zoneFile << index << "  " << step.connection.type << "  " << step.fromZoneName << " -> " << step.toZoneName
                     << "  cost=" << step.connection.cost;
            if (!homePointHop.empty()) {
                zoneFile << homePointHop;
            }
            if (!step.connection.requiresText.empty()) {
                zoneFile << "  requires=" << step.connection.requiresText;
            }
            if (!step.connection.meta.empty()) {
                zoneFile << "  meta=" << step.connection.meta;
            }
            zoneFile << '\n';
        }
        zoneFile << "---\n";
        zoneFile.close();
        std::cout << "Wrote zone itinerary to route_zones.txt\n";
    }

    std::size_t walkLegIndex = 0;
    for (std::size_t index = 0; index < expanded.legs.size(); ++index) {
        const WorldRouteLeg& leg = expanded.legs[index];
        if (leg.kind != WorldRouteLeg::Kind::Walk) {
            continue;
        }

        if (!fileOutput) {
            ++walkLegIndex;
            continue;
        }

        std::ostringstream filename;
        filename << "route_walk_" << walkLegIndex << ".lua";
        std::ofstream walkFile(filename.str());
        if (walkFile) {
            if (fileOutput) {
                WriteWalkLegLuaTable(walkFile, leg, 0, true);
                walkFile << '\n';
            } else {
                walkFile << std::fixed;
                walkFile << "Walk Leg " << walkLegIndex << "\n";
                walkFile << "Zone: " << leg.zoneName << "\n";
                walkFile << "From: " << leg.fromLabel << "\n";
                walkFile << "To:   " << leg.toLabel << "\n";
                walkFile << "Waypoints: " << leg.waypoints.size() << "\n";
                walkFile << "---\n";
                for (std::size_t wpIndex = 0; wpIndex < leg.waypoints.size(); ++wpIndex) {
                    const WorldWalkWaypoint& wp = leg.waypoints[wpIndex];
                    walkFile << wpIndex << "  " << wp.x << ", " << wp.y << ", " << wp.z;
                    if (wp.flags != 0) {
                        walkFile << "  flags=" << static_cast<int>(wp.flags);
                    }
                    walkFile << '\n';
                }
                walkFile << "---\n";
            }
            walkFile.close();
            std::cout << "Wrote walk leg " << walkLegIndex << " to " << filename.str() << "\n";
        }

        ++walkLegIndex;
    }
}
