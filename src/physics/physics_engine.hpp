#pragma once

#include <cstdint>
#include <memory>

namespace voxel {

struct PhysicsStats {
    const char* backend{"Disabled"};
    std::uint32_t bodyCount{};
    std::uint32_t activeBodyCount{};
    std::uint64_t simulationTick{};
};

class PhysicsEngine final {
public:
    PhysicsEngine();
    ~PhysicsEngine();

    PhysicsEngine(const PhysicsEngine&) = delete;
    PhysicsEngine& operator=(const PhysicsEngine&) = delete;

    void update(float deltaSeconds);
    [[nodiscard]] const PhysicsStats& stats() const noexcept { return stats_; }

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
    PhysicsStats stats_{};
    double accumulatedTime_{};
};

} // namespace voxel

