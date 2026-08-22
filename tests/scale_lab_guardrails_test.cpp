#include "planet/scale_lab_guardrails.hpp"

#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "scale-lab guardrail failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

} // namespace

int main() {
    const auto production = voxel::estimateScaleLabBudget(512U, 32U, 512U, 256U,
                                                           8192U, 128U, 96U, 0U);
    const auto lab = voxel::estimateScaleLabBudget(1024U, 32U, 1024U, 512U,
                                                    8192U, 128U, 48U,
                                                    13'981'040U);
    const auto oversized = voxel::estimateScaleLabBudget(4096U, 32U, 4096U, 2048U,
                                                          8192U, 128U, 48U, 0U);
    require(production.surfaceTiles == 2'621'442ULL,
            "production f512 tile count changed");
    require(lab.surfaceTiles == 10'485'762ULL, "f1024 tile count is not exact");
    require(lab.virtualVoxelCells == 335'544'384ULL,
            "f1024 radial payload was silently reduced");
    require(lab.accepted(), "authorized f1024/R2 configuration should pass preflight");
    require(lab.gpuMetadataBytes <= voxel::kScaleLabGpuMetadataLimit,
            "f1024 GPU metadata exceeds 1.5 GiB");
    require(!oversized.accepted() && !oversized.offsetsFit32Bit,
            "32-bit offset overflow must be rejected");
    require(!voxel::estimateScaleLabBudget(1024U, 32U, 1024U, 512U, 8192U,
                                            132U, 48U, 0U)
                 .pushConstantsFitPortableLimit,
            "push constants above the portable 128-byte limit must be rejected");
    std::cout << "production: " << production.report() << '\n'
              << "lab: " << lab.report() << '\n'
              << "oversized: " << oversized.report() << '\n';
}
