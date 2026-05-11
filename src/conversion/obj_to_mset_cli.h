#pragma once

#include <filesystem>
#include <string>
#include <vector>

struct ConversionJob {
    std::filesystem::path inputObj;
    std::filesystem::path outputBin;
};

// Resolves CLI input paths into conversion jobs.
// Returns false and sets outError on invalid input or filesystem failures.
bool BuildConversionJobs(bool                               batchMode,
                         const std::filesystem::path&       arg1,
                         const std::filesystem::path&       arg2,
                         std::vector<ConversionJob>&        outJobs,
                         std::string&                       outError);
