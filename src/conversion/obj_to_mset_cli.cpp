#include "obj_to_mset_cli.h"

#include <algorithm>
#include <sstream>
#include <system_error>

namespace {

std::string QuotePath(const std::filesystem::path& p)
{
    return std::string("\"") + p.string() + "\"";
}

} // namespace

bool BuildConversionJobs(bool                         batchMode,
                         const std::filesystem::path& arg1,
                         const std::filesystem::path& arg2,
                         std::vector<ConversionJob>&  outJobs,
                         std::string&                 outError)
{
    outJobs.clear();
    outError.clear();

    std::error_code ec;

    if (batchMode) {
        if (!std::filesystem::exists(arg1, ec) || !std::filesystem::is_directory(arg1, ec)) {
            outError = "--batch expects <obj_dir> as a directory, got " + QuotePath(arg1);
            return false;
        }

        if (!std::filesystem::create_directories(arg2, ec) && ec) {
            outError = "cannot create output directory " + QuotePath(arg2) + ": " + ec.message();
            return false;
        }

        std::vector<std::filesystem::path> entries;
        std::filesystem::directory_iterator it(arg1, ec);
        if (ec) {
            outError = "cannot open input directory " + QuotePath(arg1) + ": " + ec.message();
            return false;
        }

        for (const auto& entry : it) {
            if (!entry.is_regular_file(ec) || ec) continue;
            if (entry.path().extension() == ".obj") {
                entries.push_back(entry.path());
            }
        }

        std::sort(entries.begin(), entries.end());
        for (const auto& objPath : entries) {
            outJobs.push_back({objPath, arg2 / objPath.stem().concat(".bin")});
        }

        if (outJobs.empty()) {
            outError = "no .obj files found in input directory " + QuotePath(arg1);
            return false;
        }

        return true;
    }

    if (!std::filesystem::exists(arg1, ec) || !std::filesystem::is_regular_file(arg1, ec)) {
        outError = "single-file mode expects <input.obj> as a file, got " + QuotePath(arg1);
        return false;
    }
    if (arg1.extension() != ".obj") {
        outError = "single-file mode expects .obj input, got " + QuotePath(arg1);
        return false;
    }

    const std::filesystem::path parent = arg2.parent_path();
    if (!parent.empty()) {
        if (!std::filesystem::create_directories(parent, ec) && ec) {
            outError = "cannot create output parent directory " + QuotePath(parent) + ": " + ec.message();
            return false;
        }
    }

    outJobs.push_back({arg1, arg2});
    return true;
}
