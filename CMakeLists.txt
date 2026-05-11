cmake_minimum_required(VERSION 3.20)
project(VanavDiel LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

option(BUILD_BIN_CONVERT "Build binary: OBJ to navmesh converter" ON)
option(BUILD_BIN_ROUTING_CLI "Build binary: routing CLI" ON)
option(BUILD_BIN_DEBUG_TOOL "Build binary: debug connectivity tool" ON)
option(BUILD_BIN_VISUALIZER "Build binary: navmesh visualizer" ON)
option(BUILD_UNIT_TESTS "Build unit tests" ON)

set(VANAV_MSET_DIR "${CMAKE_SOURCE_DIR}/MSETs" CACHE PATH "Directory containing generated .bin navmeshes")
set(SQLITE3_LIBRARY /lib/x86_64-linux-gnu/libsqlite3.so.0)

set(WARN_FLAGS
  $<$<CXX_COMPILER_ID:MSVC>:/W4>
  $<$<NOT:$<CXX_COMPILER_ID:MSVC>>:-Wall -Wextra -Wpedantic>
)

include(FetchContent)
include(CTest)
enable_testing()

if (NOT TARGET Detour)
  set(RECASTNAVIGATION_DEMO OFF CACHE BOOL "" FORCE)
  set(RECASTNAVIGATION_TESTS OFF CACHE BOOL "" FORCE)
  set(RECASTNAVIGATION_EXAMPLES OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    recastnavigation
    GIT_REPOSITORY https://github.com/recastnavigation/recastnavigation.git
    GIT_TAG v1.6.0
  )
  FetchContent_MakeAvailable(recastnavigation)
endif()

if (BUILD_BIN_VISUALIZER)
  set(BUILD_EXAMPLES OFF CACHE BOOL "" FORCE)
  set(BUILD_GAMES OFF CACHE BOOL "" FORCE)
  set(BUILD_TESTING OFF CACHE BOOL "" FORCE)
  FetchContent_Declare(
    raylib
    GIT_REPOSITORY https://github.com/raysan5/raylib.git
    GIT_TAG 5.5
  )
  FetchContent_MakeAvailable(raylib)
endif()

if (BUILD_UNIT_TESTS)
  FetchContent_Declare(
    Catch2
    GIT_REPOSITORY https://github.com/catchorg/Catch2.git
    GIT_TAG v3.6.0
  )
  FetchContent_MakeAvailable(Catch2)
endif()

if (BUILD_BIN_ROUTING_CLI AND NOT EXISTS ${SQLITE3_LIBRARY})
  message(FATAL_ERROR "Could not find libsqlite3 at ${SQLITE3_LIBRARY}")
endif()

add_library(vanav_navmesh_core STATIC
  src/conversion/mesh_io.cpp
)
target_include_directories(vanav_navmesh_core PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/conversion
)
target_link_libraries(vanav_navmesh_core PUBLIC Detour)

add_library(vanav_navmesh_debug STATIC
  src/debugging/navmesh_debug_common.cpp
)
target_include_directories(vanav_navmesh_debug PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/debugging
)
target_link_libraries(vanav_navmesh_debug PUBLIC Detour)

add_library(vanav_navmesh_debug_tool STATIC
  src/debugging/navmesh_debug_tool.cpp
)
target_include_directories(vanav_navmesh_debug_tool PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/debugging
)
target_link_libraries(vanav_navmesh_debug_tool PUBLIC
  Detour
  vanav_navmesh_debug
)

add_library(vanav_zone_router STATIC
  src/pathfinding/routing/zone_route.cpp
)
target_include_directories(vanav_zone_router PUBLIC
  ${CMAKE_SOURCE_DIR}/src
  ${CMAKE_SOURCE_DIR}/src/pathfinding
  ${CMAKE_SOURCE_DIR}/src/pathfinding/routing
  ${CMAKE_SOURCE_DIR}/src/conversion
)
target_link_libraries(vanav_zone_router PUBLIC
  Detour
  vanav_navmesh_core
)

if (BUILD_BIN_CONVERT)
  add_executable(vanav_convert
    src/conversion/obj_to_mset.cpp
    src/conversion/obj_to_mset_cli.cpp
    src/conversion/generation/navmesh_generation_common.cpp
    src/conversion/generation/navmesh_generation_monolithic.cpp
    src/conversion/generation/navmesh_generation_tiled.cpp
    src/conversion/obj_preprocessing.cpp
  )
  target_include_directories(vanav_convert PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/conversion
  )
  target_link_libraries(vanav_convert PRIVATE Recast Detour vanav_navmesh_core)
  target_compile_options(vanav_convert PRIVATE ${WARN_FLAGS})
endif()

if (BUILD_BIN_ROUTING_CLI)
  add_executable(vanav_query
    src/pathfinding/query_cli.cpp
    src/pathfinding/routing/world_route.cpp
  )
  target_include_directories(vanav_query PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/pathfinding
    ${CMAKE_SOURCE_DIR}/src/pathfinding/routing
    ${CMAKE_SOURCE_DIR}/src/conversion
  )
  target_link_libraries(vanav_query PRIVATE Detour ${SQLITE3_LIBRARY} vanav_zone_router)
  target_compile_options(vanav_query PRIVATE ${WARN_FLAGS})
endif()

if (BUILD_BIN_DEBUG_TOOL)
  add_executable(vanav_debug
    src/debugging/navmesh_debug_cli.cpp
  )
  target_include_directories(vanav_debug PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/debugging
    ${CMAKE_SOURCE_DIR}/src/pathfinding
    ${CMAKE_SOURCE_DIR}/src/pathfinding/routing
  )
  target_link_libraries(vanav_debug PRIVATE vanav_zone_router vanav_navmesh_debug_tool)
  target_compile_options(vanav_debug PRIVATE ${WARN_FLAGS})
endif()

if (BUILD_BIN_VISUALIZER)
  add_executable(vanav_viewer
    src/visualization/main.cpp
    src/visualization/viz.cpp
  )
  target_include_directories(vanav_viewer PRIVATE
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/visualization
    ${CMAKE_SOURCE_DIR}/src/pathfinding
    ${CMAKE_SOURCE_DIR}/src/pathfinding/routing
    ${CMAKE_SOURCE_DIR}/src/debugging
    ${CMAKE_SOURCE_DIR}/src/conversion
  )
  target_link_libraries(vanav_viewer PRIVATE Detour raylib vanav_zone_router vanav_navmesh_debug)
  target_compile_options(vanav_viewer PRIVATE ${WARN_FLAGS})
endif()

if (BUILD_UNIT_TESTS)
  add_executable(vanav_tests
    test/test_mog_house_reachability.cpp
    test/test_jeuno_reachability.cpp
    test/test_obj_to_mset_cli.cpp
    src/conversion/obj_to_mset_cli.cpp
  )
  target_include_directories(vanav_tests PRIVATE
    ${CMAKE_SOURCE_DIR}/test
    ${CMAKE_SOURCE_DIR}/src
    ${CMAKE_SOURCE_DIR}/src/conversion
    ${CMAKE_SOURCE_DIR}/src/pathfinding
    ${CMAKE_SOURCE_DIR}/src/pathfinding/routing
  )
  target_link_libraries(vanav_tests PRIVATE
    Detour
    Catch2::Catch2WithMain
    vanav_navmesh_core
    vanav_zone_router
  )
  target_compile_definitions(vanav_tests PRIVATE
    NAVMESH_DIR="${VANAV_MSET_DIR}"
  )
  target_compile_options(vanav_tests PRIVATE ${WARN_FLAGS})

  include(Catch)
  catch_discover_tests(vanav_tests)
endif()
