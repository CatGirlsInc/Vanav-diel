// obj_to_mset.cpp
// Converts FFXI collision .obj files to Recast/Detour navigation mesh binary
// files (.bin) in the MSET format consumed by FFXIPathfinding.
//
// Preprocessing pipeline applied before Recast voxelization:
//   1. WeldVertices        -- merge near-coincident vertices within tolerance
//   2. WeldShortEdges      -- collapse tiny residual seam edges (optional)
//   3. NormalizeWinding    -- flip triangles to face upward
//   4. PruneSmallComponents -- remove small disconnected islands (if enabled)

#include "mesh_io.h"
#include "generation/navmesh_generation_common.h"
#include "generation/navmesh_generation_monolithic.h"
#include "generation/navmesh_generation_tiled.h"
#include "obj_preprocessing.h"
#include "obj_to_mset_cli.h"

#include <DetourAlloc.h>
#include <DetourNavMesh.h>

#include <atomic>
#include <filesystem>
#include <future>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

// ---- Zone transition metadata ----------------------------------------------

static void LogZoneTransitionSource(const std::filesystem::path& objPath)
{
    const auto connPath = objPath.parent_path() /
                          (objPath.stem().string() + "_connections.json");
    std::error_code ec;
    if (std::filesystem::exists(connPath, ec)) {
        std::cerr << "  [found] " << connPath.filename().string()
                  << " (zone transition metadata source)\n";
    }
}

// ---- Single-file conversion ------------------------------------------------

static bool ConvertFile(const std::filesystem::path& objPath,
                        const std::filesystem::path& binPath,
                        const BuildConfig&            cfg)
{
    ObjMesh mesh;
    if (!LoadObj(objPath, mesh)) return false;

    // 1. Weld near-coincident vertices
    const WeldStats weldStats = WeldVertices(mesh, cfg.weldTolerance);
    if (weldStats.mergedVerts > 0) {
        std::cerr << "  [weld] " << weldStats.originalVerts << " -> "
                  << weldStats.finalVerts << " verts ("
                  << weldStats.mergedVerts << " merged, tol="
                  << cfg.weldTolerance << ")\n";
    }

    // 2. Collapse very short edges (optional)
    if (cfg.edgeWeldTolerance > 0.0f) {
        const EdgeWeldStats edgeStats = WeldShortEdges(mesh, cfg.edgeWeldTolerance);
        std::cerr << "  [edge weld] " << edgeStats.originalVerts << " -> "
                  << edgeStats.finalVerts << " verts, "
                  << edgeStats.originalTris << " -> " << edgeStats.finalTris
                  << " tris (collapsedEdges=" << edgeStats.collapsedEdges
                  << ", removedDegenerateTris=" << edgeStats.removedDegenerateTris
                  << ", tol=" << cfg.edgeWeldTolerance << ")\n";
    }

    // 3. Normalize winding
    const int flipped = NormalizeWinding(mesh);
    if (flipped) {
        std::cerr << "  [winding] flipped " << flipped << " triangles\n";
    }

    // 4. Prune small disconnected components (optional)
    if (cfg.minComponentTris > 0) {
        const ComponentPruneStats ps =
            PruneSmallDisconnectedComponents(mesh, cfg.minComponentTris);
        std::cerr << "  [component prune] kept " << ps.keptTris
                  << " tris across " << ps.keptComponents << "/"
                  << ps.totalComponents << " components"
                  << " (min=" << cfg.minComponentTris
                  << ", pruned=" << ps.prunedTris << " tris in "
                  << ps.prunedComponents << " components)\n";
        if (mesh.tris.empty()) {
            std::cerr << "All triangles pruned\n";
            return false;
        }
    }

    dtNavMesh* navMesh = nullptr;
    float usedCellSize = cfg.cs;
    int usedMinNavComponentPolys = cfg.minNavComponentPolys;
    bool usedTiledFallback = false;

    const float fallbackSizes[] = {cfg.cs, 0.45f, 0.50f, 0.55f, 0.60f};
    for (const float cs : fallbackSizes) {
        const auto pruneThresholds =
            BuildNavComponentPruneThresholds(cfg.minNavComponentPolys);

        for (const int minPolys : pruneThresholds) {
            if (GenerateMonolithicNavMesh(mesh, cfg, cs, minPolys, &navMesh)) {
                usedCellSize = cs;
                usedMinNavComponentPolys = minPolys;
                break;
            }
        }
        if (navMesh) break;

        // First-attempt fallback: keep requested cs and try tiled generation
        // before increasing cs.
        if (cs == cfg.cs) {
            std::cerr << "  [fallback] trying tiled generation at cs=" << cs << "\n";
            for (const int minPolys : pruneThresholds) {
                if (GenerateTiledNavMesh(mesh, cfg, cs, minPolys, &navMesh)) {
                    usedCellSize = cs;
                    usedMinNavComponentPolys = minPolys;
                    usedTiledFallback = true;
                    break;
                }
            }
            if (navMesh) break;
        }
    }

    if (!navMesh) return false;

    if (usedTiledFallback) {
        std::cerr << "  [fallback] used tiled generation\n";
    }
    if (usedCellSize != cfg.cs) {
        std::cerr << "  [fallback] used cs=" << usedCellSize
                  << " (instead of " << cfg.cs << ")\n";
    }
    if (usedMinNavComponentPolys != cfg.minNavComponentPolys) {
        std::cerr << "  [fallback] used min-nav-component-polys="
                  << usedMinNavComponentPolys
                  << " (instead of " << cfg.minNavComponentPolys << ")\n";
    }

    LogZoneTransitionSource(objPath);

    const bool ok = WriteMset(binPath, navMesh);
    dtFreeNavMesh(navMesh);
    return ok;
}

// ---- CLI -------------------------------------------------------------------

static void PrintUsage(const char* exe)
{
    std::cout
        << "Usage:\n"
        << "  " << exe << " <input.obj> <output.bin> [options]\n"
        << "  " << exe << " --batch <obj_dir> <out_dir> [options]\n\n"
        << "Examples:\n"
        << "  " << exe << " ./OBJs/la_theine_plateau.obj ./MSETs/la_theine_plateau.bin\n"
        << "  " << exe << " --batch ./OBJs ./MSETs\n\n"
        << "Options:\n"
        << "  --cs <f>                Voxel cell size             (default 0.40)\n"
        << "  --ch <f>                Voxel cell height           (default 0.20)\n"
        << "  --height <f>            Agent height                (default 1.80)\n"
        << "  --radius <f>            Agent radius                (default 0.20)\n"
        << "  --climb <f>             Agent max climb             (default 0.50)\n"
        << "  --slope <f>             Max walkable slope deg      (default 46.0)\n"
        << "  --region-min-size <n>   Min region area threshold    (default 8)\n"
        << "  --region-merge-size <n> Merge region area threshold  (default 20)\n"
        << "  --edge-max-len <f>      Max contour edge length      (default 12.0)\n"
        << "  --edge-max-error <f>    Contour simplification error (default 1.3)\n"
        << "  --verts-per-poly <n>    Max verts per poly [3..6]    (default 6)\n"
        << "  --detail-sample-dist <f> Detail sample distance mul  (default 6.0)\n"
        << "  --detail-sample-max-error <f>\n"
        << "                          Detail sample max error mul   (default 1.0)\n"
        << "  --weld-tolerance <f>    Vertex weld distance        (default 0.01)\n"
        << "  --edge-weld-tolerance <f>\n"
        << "                          Collapse short mesh edges     (default 0, off)\n"
        << "  --min-component-tris <n>\n"
        << "                          Remove disconnected components smaller\n"
        << "                          than n triangles            (default 0, off)\n"
        << "  --min-nav-component-polys <n>\n"
        << "                          Remove disconnected navmesh components\n"
        << "                          smaller than n polys        (default 11)\n"
        << "  --tile-size-cells <n>   Tiled fallback tile size     (default 160)\n"
        << "  --max-tiles <n>         Reserved for future tuning    (default 4096)\n"
        << "  --max-polys-per-tile <n>\n"
        << "                          Tiled fallback max polys/tile (default 4096)\n";
}

int main(int argc, char** argv)
{
    if (argc < 2) {
        PrintUsage(argv[0]);
        return 1;
    }

    BuildConfig cfg;
    bool batchMode = false;
    std::filesystem::path arg1, arg2;

    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--batch") {
            batchMode = true;
        } else if (a == "--cs" && i + 1 < argc) {
            cfg.cs = std::stof(argv[++i]);
        } else if (a == "--ch" && i + 1 < argc) {
            cfg.ch = std::stof(argv[++i]);
        } else if (a == "--height" && i + 1 < argc) {
            cfg.agentHeight = std::stof(argv[++i]);
        } else if (a == "--radius" && i + 1 < argc) {
            cfg.agentRadius = std::stof(argv[++i]);
        } else if (a == "--climb" && i + 1 < argc) {
            cfg.agentClimb = std::stof(argv[++i]);
        } else if (a == "--slope" && i + 1 < argc) {
            cfg.slopeAngle = std::stof(argv[++i]);
        } else if (a == "--region-min-size" && i + 1 < argc) {
            cfg.regionMinSize = std::stoi(argv[++i]);
            if (cfg.regionMinSize < 0) {
                std::cerr << "--region-min-size must be >= 0\n";
                return 1;
            }
        } else if (a == "--region-merge-size" && i + 1 < argc) {
            cfg.regionMergeSize = std::stoi(argv[++i]);
            if (cfg.regionMergeSize < 0) {
                std::cerr << "--region-merge-size must be >= 0\n";
                return 1;
            }
        } else if (a == "--edge-max-len" && i + 1 < argc) {
            cfg.edgeMaxLen = std::stof(argv[++i]);
            if (cfg.edgeMaxLen <= 0.0f) {
                std::cerr << "--edge-max-len must be > 0\n";
                return 1;
            }
        } else if (a == "--edge-max-error" && i + 1 < argc) {
            cfg.edgeMaxError = std::stof(argv[++i]);
            if (cfg.edgeMaxError < 0.0f) {
                std::cerr << "--edge-max-error must be >= 0\n";
                return 1;
            }
        } else if (a == "--verts-per-poly" && i + 1 < argc) {
            cfg.vertsPerPoly = std::stoi(argv[++i]);
            if (cfg.vertsPerPoly < 3 || cfg.vertsPerPoly > 6) {
                std::cerr << "--verts-per-poly must be between 3 and 6\n";
                return 1;
            }
        } else if (a == "--detail-sample-dist" && i + 1 < argc) {
            cfg.detailSampleDist = std::stof(argv[++i]);
            if (cfg.detailSampleDist < 0.0f) {
                std::cerr << "--detail-sample-dist must be >= 0\n";
                return 1;
            }
        } else if (a == "--detail-sample-max-error" && i + 1 < argc) {
            cfg.detailSampleMaxErr = std::stof(argv[++i]);
            if (cfg.detailSampleMaxErr < 0.0f) {
                std::cerr << "--detail-sample-max-error must be >= 0\n";
                return 1;
            }
        } else if (a == "--weld-tolerance" && i + 1 < argc) {
            cfg.weldTolerance = std::stof(argv[++i]);
            if (cfg.weldTolerance < 0.0f) {
                std::cerr << "--weld-tolerance must be >= 0\n";
                return 1;
            }
        } else if (a == "--edge-weld-tolerance" && i + 1 < argc) {
            cfg.edgeWeldTolerance = std::stof(argv[++i]);
            if (cfg.edgeWeldTolerance < 0.0f) {
                std::cerr << "--edge-weld-tolerance must be >= 0\n";
                return 1;
            }
        } else if (a == "--min-component-tris" && i + 1 < argc) {
            cfg.minComponentTris = std::stoi(argv[++i]);
            if (cfg.minComponentTris < 0) {
                std::cerr << "--min-component-tris must be >= 0\n";
                return 1;
            }
        } else if (a == "--min-nav-component-polys" && i + 1 < argc) {
            cfg.minNavComponentPolys = std::stoi(argv[++i]);
            if (cfg.minNavComponentPolys < 0) {
                std::cerr << "--min-nav-component-polys must be >= 0\n";
                return 1;
            }
        } else if (a == "--tile-size-cells" && i + 1 < argc) {
            cfg.tileSizeCells = std::stoi(argv[++i]);
            if (cfg.tileSizeCells < 32) {
                std::cerr << "--tile-size-cells must be >= 32\n";
                return 1;
            }
        } else if (a == "--max-tiles" && i + 1 < argc) {
            cfg.maxTiles = std::stoi(argv[++i]);
            if (cfg.maxTiles < 1) {
                std::cerr << "--max-tiles must be >= 1\n";
                return 1;
            }
        } else if (a == "--max-polys-per-tile" && i + 1 < argc) {
            cfg.maxPolysPerTile = std::stoi(argv[++i]);
            if (cfg.maxPolysPerTile < 1) {
                std::cerr << "--max-polys-per-tile must be >= 1\n";
                return 1;
            }
        } else if (arg1.empty()) {
            arg1 = a;
        } else if (arg2.empty()) {
            arg2 = a;
        }
    }

    if (arg1.empty() || arg2.empty()) {
        PrintUsage(argv[0]);
        return 1;
    }

    std::vector<ConversionJob> jobs;
    std::string planError;
    if (!BuildConversionJobs(batchMode, arg1, arg2, jobs, planError)) {
        std::cerr << planError << '\n';
        return 1;
    }

    if (batchMode) {
        const std::size_t entryCount = jobs.size();
        const unsigned int nThreads = std::max(
            1u,
            std::min(std::thread::hardware_concurrency() / 2u,
                     static_cast<unsigned int>(entryCount)));

        std::atomic<int> ok{0}, fail{0};
        std::mutex printMtx;

        const std::size_t windowSize = nThreads;
        std::vector<std::future<bool>> window;
        window.reserve(windowSize);

        auto drainOne = [&]() {
            if (window.empty()) return;
            if (window.front().get()) {
                ++ok;
            } else {
                ++fail;
            }
            window.erase(window.begin());
        };

        for (const auto& job : jobs) {
            if (window.size() >= windowSize) drainOne();

            window.push_back(std::async(std::launch::async, [job, &cfg, &printMtx]() -> bool {
                const bool result = ConvertFile(job.inputObj, job.outputBin, cfg);
                std::lock_guard<std::mutex> lk(printMtx);
                if (result) {
                    std::cout << "  wrote " << job.outputBin.filename().string() << '\n';
                } else {
                    std::cerr << job.inputObj.filename().string() << " FAILED\n";
                }
                return result;
            }));
        }
        while (!window.empty()) drainOne();

        std::cout << "\nDone: " << ok.load() << " succeeded, "
                  << fail.load() << " failed.\n";
        return fail.load() > 0 ? 1 : 0;
    }

    const ConversionJob& job = jobs.front();
    const bool ok = ConvertFile(job.inputObj, job.outputBin, cfg);
    if (ok) std::cout << "  wrote " << job.outputBin.filename().string() << '\n';
    return ok ? 0 : 1;
}
