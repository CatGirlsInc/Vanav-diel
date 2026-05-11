#pragma once

#include <filesystem>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

struct WorldZone {
    int zoneId = 0;
    std::string name;
    std::string canon;
};

struct WorldNpc {
    int npcid = 0;
    std::string name;
    std::string polutilsName;
    int zoneId = 0;
    std::string zoneName;
    float posX = 0.0f;
    float posY = 0.0f;
    float posZ = 0.0f;
    int posRot = 0;
    std::string contentTag;
};

struct WorldConnection {
    int id = 0;
    std::string type;
    int fromZoneId = 0;
    float fromX = 0.0f;
    float fromY = 0.0f;
    float fromZ = 0.0f;
    float fromRot = 0.0f;
    int toZoneId = 0;
    float toX = 0.0f;
    float toY = 0.0f;
    float toZ = 0.0f;
    float toRot = 0.0f;
    float cost = 1.0f;
    std::string requiresText;
    std::string meta;
};

struct WorldGraph {
    std::unordered_map<int, WorldZone> zonesById;
    std::unordered_map<std::string, int> zoneIdsByNormalizedName;
    std::vector<WorldNpc> npcs;
    std::vector<WorldConnection> connections;
    std::unordered_map<int, std::vector<std::size_t>> outgoingByZoneId;
};

struct WorldTarget {
    WorldNpc npc;
    std::string matchedLabel;
    std::string matchKind;
    int matchScore = 0;
};

struct WorldRouteStep {
    WorldConnection connection;
    std::string fromZoneName;
    std::string toZoneName;
};

struct WorldWalkWaypoint {
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    unsigned char flags = 0;
};

struct WorldRouteLeg {
    enum class Kind {
        Walk,
        Warp,
    };

    Kind kind = Kind::Walk;
    int zoneId = 0;
    std::string zoneName;
    std::string label;
    std::string fromLabel;
    std::string toLabel;
    WorldConnection connection;
    std::vector<WorldWalkWaypoint> waypoints;
    float startPos[3] = {0.0f, 0.0f, 0.0f};
    float endPos[3] = {0.0f, 0.0f, 0.0f};
    float cost = 0.0f;
};

struct WorldRoute {
    WorldTarget start;
    WorldTarget end;
    std::vector<WorldRouteStep> steps;
    std::vector<WorldRouteLeg> legs;
    float totalCost = 0.0f;
};

bool LoadWorldGraph(const std::filesystem::path& dbPath, WorldGraph* outGraph, std::string* outError);

std::optional<WorldTarget> ResolveWorldTarget(
    const WorldGraph& graph,
    const std::string& query,
    std::vector<std::string>* outWarnings = nullptr);

std::optional<WorldRoute> FindWorldRoute(
    const WorldGraph& graph,
    const WorldTarget& start,
    const WorldTarget& end,
    std::vector<std::string>* outWarnings = nullptr);

void PrintWorldRoute(
    const WorldRoute& route,
    const WorldGraph& graph,
    const std::filesystem::path& navMeshDir,
    bool fileOutput,
    std::vector<std::string>* outWarnings = nullptr);
