// test_mog_house_reachability.cpp
// Verifies that every zone-line entrance in each starting city can reach that
// city's mog house door via an in-zone navmesh path.
//
// Zone data sourced from *_connections.json files alongside the .obj exports.
// `from_pos` on a non-mog-house connection = where you arrive in this zone.
// `from_pos` on the mog house connection   = the entrance trigger in this zone.
//
// Build-time constant NAVMESH_DIR must point to the directory containing the
// .bin navmesh files (set by CMake; defaults to the sibling MSETs directory).

#include "navmesh_test_utils.h"
#include "mesh_io.h"

#include <catch2/catch_test_macros.hpp>

#include <DetourNavMesh.h>
#include <DetourAlloc.h>

#include <filesystem>

#ifndef NAVMESH_DIR
#  define NAVMESH_DIR "MSETs"
#endif

// ── Helper ────────────────────────────────────────────────────────────────────

static int PathLength(const std::filesystem::path& bin,
                      const float*                 from,
                      const float*                 to)
{
    dtNavMesh* nav = nullptr;
    if (!LoadMset(bin, &nav)) return 0;
    const auto waypoints = QueryPath(nav, from, to);
    dtFreeNavMesh(nav);
    return static_cast<int>(waypoints.size());
}

// ── Bastok Mines (zone 234) ───────────────────────────────────────────────────
// Mog house entrance (zmrg → zmrh): (121.831, 3.367, 71.955)
// Zone lines:
//   z6i0  South Gustaberg entrance :  (-15.961,  5.475, 136.018)
//   z6i2  Bastok Markets entrance  : (-104.080, -8.101, -84.578)
//   z6i4  Zeruhn Mines entrance    : (-196.032, 10.599,  21.883)

TEST_CASE("Bastok Mines – zone lines reach mog house", "[bastok][mog_house]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "bastok_mines.bin";
    const float mogHouse[3] = {121.831f, 3.367f, 71.955f};

    SECTION("from South Gustaberg entrance (z6i0)")
    {
        const float from[3] = {-15.961f, 5.475f, 136.018f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }

    SECTION("from Bastok Markets entrance (z6i2)")
    {
        const float from[3] = {-104.080f, -8.101f, -84.578f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }

    SECTION("from Zeruhn Mines entrance (z6i4)")
    {
        const float from[3] = {-196.032f, 10.599f, 21.883f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }
}

// ── Southern San d'Oria (zone 230) ───────────────────────────────────────────
// Mog house entrance (zmr0 → zmr1): (164.933, 5.547, -164.792)
// Zone lines:
//   z6e0  East Ronfaure entrance      :  (113.458,  4.079,   57.351)
//   z6e2  West Ronfaure entrance      : (-113.372,  4.075,   57.418)
//   z6e4  Northern San d'Oria entrance: (   0.013,  8.469,  -57.965)

TEST_CASE("Southern San d'Oria – zone lines reach mog house", "[sandoria][mog_house]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "southern_san_d_oria.bin";
    const float mogHouse[3] = {164.933f, 5.547f, -164.792f};

    SECTION("from East Ronfaure entrance (z6e0)")
    {
        const float from[3] = {113.458f, 4.079f, 57.351f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }

    SECTION("from West Ronfaure entrance (z6e2)")
    {
        const float from[3] = {-113.372f, 4.075f, 57.418f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }

    SECTION("from Northern San d'Oria entrance (z6e4)")
    {
        const float from[3] = {0.013f, 8.469f, -57.965f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }
}

// ── Windurst Waters (zone 238) ────────────────────────────────────────────────
// Mog house entrance (zmr6 → zmr7): (159.934, 8.386, 64.086)
// Zone lines:
//   z6m0  West Sarutabaruta entrance: ( -40.194, 7.871, -248.809)
//   z6m2  Port Windurst entrance    : ( -59.591, 7.115,  209.045)
//   z6m4  Windurst Walls entrance   : ( 160.108, 7.303,  -60.360)

TEST_CASE("Windurst Waters – zone lines reach mog house", "[windurst][mog_house]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "windurst_waters.bin";
    const float mogHouse[3] = {159.934f, 8.386f, 64.086f};

    SECTION("from West Sarutabaruta entrance (z6m0)")
    {
        const float from[3] = {-40.194f, 7.871f, -248.809f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }

    SECTION("from Port Windurst entrance (z6m2)")
    {
        const float from[3] = {-59.591f, 7.115f, 209.045f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }

    SECTION("from Windurst Walls entrance (z6m4)")
    {
        const float from[3] = {160.108f, 7.303f, -60.360f};
        REQUIRE(PathLength(bin, from, mogHouse) > 0);
    }
}
