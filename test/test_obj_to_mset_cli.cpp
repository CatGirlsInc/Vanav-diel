#include "obj_to_mset_cli.h"

#include <catch2/catch_test_macros.hpp>

#include <filesystem>
#include <fstream>
#include <string>
#include <vector>

namespace {

struct TempDir {
    std::filesystem::path path;

    explicit TempDir(const std::string& suffix)
    {
        path = std::filesystem::temp_directory_path() /
               ("ffxi_obj_to_mset_cli_" + suffix);
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
        std::filesystem::create_directories(path, ec);
    }

    ~TempDir()
    {
        std::error_code ec;
        std::filesystem::remove_all(path, ec);
    }
};

static void TouchFile(const std::filesystem::path& file)
{
    std::error_code ec;
    std::filesystem::create_directories(file.parent_path(), ec);
    std::ofstream out(file);
    out << "dummy";
}

} // namespace

TEST_CASE("single-file mode builds exactly one conversion job", "[obj_to_mset][cli]")
{
    TempDir temp("single");
    const std::filesystem::path inObj = temp.path / "input" / "zone.obj";
    const std::filesystem::path outBin = temp.path / "out" / "zone.bin";
    TouchFile(inObj);

    std::vector<ConversionJob> jobs;
    std::string error;
    const bool ok = BuildConversionJobs(false, inObj, outBin, jobs, error);

    REQUIRE(ok);
    REQUIRE(error.empty());
    REQUIRE(jobs.size() == 1);
    CHECK(jobs[0].inputObj == inObj);
    CHECK(jobs[0].outputBin == outBin);
    CHECK(std::filesystem::is_directory(outBin.parent_path()));
}

TEST_CASE("batch mode builds jobs from obj directory", "[obj_to_mset][cli]")
{
    TempDir temp("batch");
    const std::filesystem::path inDir = temp.path / "objs";
    const std::filesystem::path outDir = temp.path / "msets";

    TouchFile(inDir / "b_zone.obj");
    TouchFile(inDir / "a_zone.obj");
    TouchFile(inDir / "readme.txt");

    std::vector<ConversionJob> jobs;
    std::string error;
    const bool ok = BuildConversionJobs(true, inDir, outDir, jobs, error);

    REQUIRE(ok);
    REQUIRE(error.empty());
    REQUIRE(jobs.size() == 2);
    CHECK(jobs[0].inputObj.filename() == "a_zone.obj");
    CHECK(jobs[0].outputBin.filename() == "a_zone.bin");
    CHECK(jobs[1].inputObj.filename() == "b_zone.obj");
    CHECK(jobs[1].outputBin.filename() == "b_zone.bin");
    CHECK(std::filesystem::is_directory(outDir));
}

TEST_CASE("batch mode rejects file path input cleanly", "[obj_to_mset][cli]")
{
    TempDir temp("batch_invalid");
    const std::filesystem::path inFile = temp.path / "single.obj";
    const std::filesystem::path outDir = temp.path / "msets";
    TouchFile(inFile);

    std::vector<ConversionJob> jobs;
    std::string error;
    const bool ok = BuildConversionJobs(true, inFile, outDir, jobs, error);

    REQUIRE_FALSE(ok);
    REQUIRE_FALSE(error.empty());
    CHECK(jobs.empty());
}
