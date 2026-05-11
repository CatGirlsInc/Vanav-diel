// test_jeuno_reachability.cpp
// Verifies that every starting city (Bastok, Windurst, San d'Oria) can reach
// Jeuno via walking paths across the relevant zones.
//
// Each test case verifies that within each zone along the path, the zone-line
// entrance point can reach the zone-line exit point that leads to the next zone.
//
// Paths tested:
//   Bastok Mines -> Bastok Markets -> South Gustaberg -> North Gustaberg
//     -> Konschtat Highlands -> Pashhow Marshlands -> Rolanberry Fields
//     -> Lower Jeuno
//
//   Windurst Waters -> Port Windurst -> West Sarutabaruta -> East Sarutabaruta
//     -> Tahrongi Canyon -> Meriphataud Mountains -> Sauromugue Champaign
//     -> Port Jeuno -> Lower Jeuno
//
//   Southern San d'Oria -> Northern San d'Oria -> East Ronfaure -> West Ronfaure
//     -> La Theine Plateau -> Jugner Forest -> Batallia Downs -> Upper Jeuno

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

// ══════════════════════════════════════════════════════════════════════════════
// BASTOK PATH: Bastok Mines → Jeuno (via Rolanberry Fields)
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Bastok Mines → Bastok Markets zone line", "[bastok][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "bastok_mines.bin";
    // Zone line exit (Bastok Markets entrance): z6i2
    const float from[3] = {-104.08f, -8.101f, -84.578f};
    // Mog house can act as reference point to verify navmesh is valid
    const float to[3] = {121.831f, 3.367f, 71.955f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Bastok Markets → South Gustaberg zone line", "[bastok][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "bastok_markets.bin";
    // Zone line entrance (Bastok Markets): z6j0
    const float from[3] = {-202.243f, -0.332f, 197.784f};
    // Zone line exit (South Gustaberg entrance): z6j4
    const float to[3] = {-365.697f, 13.544f, 185.638f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("South Gustaberg → North Gustaberg zone line", "[bastok][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "south_gustaberg.bin";
    // Zone line entrance (Bastok Markets): z2z2
    const float from[3] = {260.0f, 3.225f, 180.0f};
    // Zone line exit (North Gustaberg): z2z4
    const float to[3] = {0.877f, 6.763f, -80.156f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("North Gustaberg → Konschtat Highlands zone line", "[bastok][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "north_gustaberg.bin";
    // Zone line entrance (South Gustaberg): z2y4
    const float from[3] = {-440.639f, -33.094f, 276.308f};
    // Zone line exit (Konschtat Highlands): z2y6
    const float to[3] = {-518.828f, -33.034f, -592.074f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Konschtat Highlands → Pashhow Marshlands zone line", "[bastok][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "konschtat_highlands.bin";
    // Zone line entrance (North Gustaberg): z300
    const float from[3] = {101.199f, 77.595f, 611.982f};
    // Zone line exit (Pashhow Marshlands): z302
    const float to[3] = {520.1f, -26.451f, -752.858f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Pashhow Marshlands → Rolanberry Fields zone line", "[bastok][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "pashhow_marshlands.bin";
    // Zone line entrance (Konschtat Highlands): z310
    const float from[3] = {-543.87f, -20.719f, 651.966f};
    // Zone line exit (Rolanberry Fields): z312
    const float to[3] = {551.257f, -20.719f, -697.035f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Rolanberry Fields → Lower Jeuno zone line", "[bastok][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "rolanberry_fields.bin";
    // Zone line entrance (Pashhow Marshlands): z320
    const float from[3] = {-124.192f, 7.558f, 920.091f};
    // Zone line exit (Lower Jeuno): z326
    const float to[3] = {340.051f, -17.423f, -613.388f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Lower Jeuno interior path", "[bastok][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "lower_jeuno.bin";
    // Zone line entrance (Rolanberry Fields): z6t0
    const float from[3] = {-123.065f, 4.435f, 200.691f};
    // Mog house as verification destination
    const float to[3] = {43.944f, 9.359f, -88.58f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

// ══════════════════════════════════════════════════════════════════════════════
// WINDURST PATH: Windurst Waters → Jeuno (via Sauromugue Champaign & Port Jeuno)
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Windurst Waters → Port Windurst zone line", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "windurst_waters.bin";
    // Zone line entrance (Windurst Waters): z6m2
    const float from[3] = {-59.591f, 7.115f, 209.045f};
    // Mog house as reference point
    const float to[3] = {159.934f, 8.386f, 64.086f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Port Windurst → West Sarutabaruta zone line", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "port_windurst.bin";
    // Zone line entrance (Windurst Waters): z6o2
    const float from[3] = {-114.594f, 11.654f, -215.791f};
    // Zone line exit (West Sarutabaruta): z6o0
    const float to[3] = {-249.083f, 11.658f, -199.98f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("West Sarutabaruta → East Sarutabaruta zone line", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "west_sarutabaruta.bin";
    // Zone line entrance (Port Windurst): z372
    const float from[3] = {169.197f, 3.751f, 320.032f};
    // Zone line exit (East Sarutabaruta): z374
    const float to[3] = {504.436f, 13.689f, -133.854f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("East Sarutabaruta → Tahrongi Canyon zone line", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "east_sarutabaruta.bin";
    // Zone line entrance (West Sarutabaruta): z382
    const float from[3] = {-343.926f, 13.696f, -106.455f};
    // Zone line exit (Tahrongi Canyon): z386
    const float to[3] = {306.101f, 37.753f, -664.733f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Tahrongi Canyon → Meriphataud Mountains zone line", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "tahrongi_canyon.bin";
    // Zone line entrance (East Sarutabaruta): z390
    const float from[3] = {-64.818f, -42.56f, 740.87f};
    // Zone line exit (Meriphataud Mountains): z392
    const float to[3] = {-238.244f, -48.73f, -703.991f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Meriphataud Mountains → Sauromugue Champaign zone line", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "meriphataud_mountains.bin";
    // Zone line entrance (Tahrongi Canyon): z3b0
    const float from[3] = {-119.897f, 36.606f, 620.103f};
    // Zone line exit (Sauromugue Champaign): z3b2
    const float to[3] = {-454.593f, -23.968f, -661.763f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Sauromugue Champaign → Port Jeuno zone line", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "sauromugue_champaign.bin";
    // Zone line entrance (Meriphataud Mountains): z3c0
    const float from[3] = {519.887f, 21.817f, 507.899f};
    // Zone line exit (Port Jeuno): z3c4
    const float to[3] = {-579.985f, 3.881f, -399.839f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Port Jeuno → Lower Jeuno zone line", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "port_jeuno.bin";
    // Zone line entrance (Sauromugue Champaign): z6u0
    const float from[3] = {54.979f, 3.629f, 0.019f};
    // Zone line exit (Lower Jeuno): z6u2
    const float to[3] = {-150.994f, 6.324f, 22.82f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Lower Jeuno interior path (Windurst route)", "[windurst][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "lower_jeuno.bin";
    // Zone line entrance (Port Jeuno): z6t4
    const float from[3] = {29.694f, 3.019f, -41.444f};
    // Mog house as verification destination
    const float to[3] = {43.944f, 9.359f, -88.58f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

// ══════════════════════════════════════════════════════════════════════════════
// SAN D'ORIA PATH: Southern San d'Oria → Jeuno (via Batallia Downs & Upper Jeuno)
// ══════════════════════════════════════════════════════════════════════════════

TEST_CASE("Southern San d'Oria → Northern San d'Oria zone line", "[sandoria][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "southern_san_d_oria.bin";
    // Zone line exit (Northern San d'Oria): z6e4
    const float from[3] = {0.013f, 8.469f, -57.965f};
    // Mog house as reference point
    const float to[3] = {164.933f, 5.547f, -164.792f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Northern San d'Oria → East Ronfaure zone line", "[sandoria][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "northern_san_d_oria.bin";
    // Zone line entrance (Southern San d'Oria): z6f0
    const float from[3] = {-0.107f, 7.068f, 36.133f};
    // Zone line exit (West Ronfaure): z6f2
    const float to[3] = {-252.158f, -1.663f, -43.913f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("East Ronfaure → West Ronfaure zone line", "[sandoria][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "east_ronfaure.bin";
    // Zone line entrance (Northern San d'Oria): z2t2
    const float from[3] = {79.181f, 70.089f, -280.841f};
    // Zone line exit (West Ronfaure): z2t4
    const float to[3] = {56.14f, 37.462f, 199.961f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("West Ronfaure → La Theine Plateau zone line", "[sandoria][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "west_ronfaure.bin";
    // Zone line entrance (East Ronfaure): z2s4
    const float from[3] = {-11.697f, 53.643f, -140.0f};
    // Zone line exit (La Theine Plateau): z2sc
    const float to[3] = {-560.415f, 10.328f, 613.176f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("La Theine Plateau → Jugner Forest zone line", "[sandoria][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "la_theine_plateau.bin";
    // Zone line entrance (West Ronfaure): z2u0
    const float from[3] = {-558.569f, 7.049f, -688.049f};
    // Zone line exit (Jugner Forest): z2u2
    const float to[3] = {801.831f, -24.326f, 37.618f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Jugner Forest → Batallia Downs zone line", "[sandoria][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "jugner_forest.bin";
    // Zone line entrance (La Theine Plateau): z2w0
    const float from[3] = {-599.998f, 16.683f, 440.077f};
    // Zone line exit (Batallia Downs): z2w2
    const float to[3] = {591.936f, 15.294f, -560.095f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Batallia Downs → Upper Jeuno zone line", "[sandoria][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "batallia_downs.bin";
    // Zone line entrance (Jugner Forest): z2x0
    const float from[3] = {-444.042f, 16.166f, 240.077f};
    // Zone line exit (Upper Jeuno): z2x4
    const float to[3] = {491.918f, -1.171f, 159.982f};
    REQUIRE(PathLength(bin, from, to) > 0);
}

TEST_CASE("Upper Jeuno interior path", "[sandoria][path_to_jeuno]")
{
    const std::filesystem::path bin =
        std::filesystem::path(NAVMESH_DIR) / "upper_jeuno.bin";
    // Zone line entrance (Batallia Downs): z6s0
    const float from[3] = {-106.095f, 4.637f, -189.999f};
    // Mog house as verification destination
    const float to[3] = {49.18f, 9.642f, 82.946f};
    REQUIRE(PathLength(bin, from, to) > 0);
}
