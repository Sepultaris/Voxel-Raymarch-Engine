#include "physics/physics_engine.hpp"

#if defined(VOXEL_HAS_JOLT)

#include <Jolt/Jolt.h>

#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayer.h>
#include <Jolt/Physics/Collision/ObjectLayer.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/RegisterTypes.h>

#include <algorithm>
#include <array>
#include <cstdarg>
#include <cstdio>
#include <iostream>
#include <thread>

namespace voxel {
namespace {

namespace ObjectLayers {
constexpr JPH::ObjectLayer kStatic = 0;
constexpr JPH::ObjectLayer kDynamic = 1;
constexpr JPH::ObjectLayer kSensor = 2;
constexpr JPH::ObjectLayer kCount = 3;
} // namespace ObjectLayers

namespace BroadPhaseLayers {
constexpr JPH::BroadPhaseLayer kStatic{0};
constexpr JPH::BroadPhaseLayer kDynamic{1};
constexpr JPH::BroadPhaseLayer kSensor{2};
constexpr JPH::uint kCount = 3;
} // namespace BroadPhaseLayers

void traceJolt(const char* format, ...) {
    std::array<char, 1024> buffer{};
    va_list arguments;
    va_start(arguments, format);
    std::vsnprintf(buffer.data(), buffer.size(), format, arguments);
    va_end(arguments);
    std::clog << "Jolt: " << buffer.data() << '\n';
}

class BroadPhaseLayerInterface final : public JPH::BroadPhaseLayerInterface {
public:
    JPH::uint GetNumBroadPhaseLayers() const override { return BroadPhaseLayers::kCount; }

    JPH::BroadPhaseLayer GetBroadPhaseLayer(JPH::ObjectLayer layer) const override {
        switch (layer) {
        case ObjectLayers::kStatic:
            return BroadPhaseLayers::kStatic;
        case ObjectLayers::kDynamic:
            return BroadPhaseLayers::kDynamic;
        case ObjectLayers::kSensor:
            return BroadPhaseLayers::kSensor;
        default:
            return BroadPhaseLayers::kStatic;
        }
    }
};

class ObjectVsBroadPhaseFilter final : public JPH::ObjectVsBroadPhaseLayerFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer objectLayer, JPH::BroadPhaseLayer broadPhaseLayer) const override {
        if (objectLayer == ObjectLayers::kStatic) {
            return broadPhaseLayer == BroadPhaseLayers::kDynamic;
        }
        if (objectLayer == ObjectLayers::kSensor) {
            return broadPhaseLayer == BroadPhaseLayers::kDynamic;
        }
        return true;
    }
};

class ObjectLayerPairFilter final : public JPH::ObjectLayerPairFilter {
public:
    bool ShouldCollide(JPH::ObjectLayer first, JPH::ObjectLayer second) const override {
        if (first == ObjectLayers::kStatic) {
            return second == ObjectLayers::kDynamic;
        }
        if (first == ObjectLayers::kSensor) {
            return second == ObjectLayers::kDynamic;
        }
        return true;
    }
};

class JoltRuntime final {
public:
    JoltRuntime() {
        JPH::RegisterDefaultAllocator();
        JPH::Trace = traceJolt;
        JPH::Factory::sInstance = new JPH::Factory();
        JPH::RegisterTypes();
    }

    ~JoltRuntime() {
        JPH::UnregisterTypes();
        delete JPH::Factory::sInstance;
        JPH::Factory::sInstance = nullptr;
    }
};

} // namespace

struct PhysicsEngine::Impl {
    Impl()
        : temporaryAllocator(32 * 1024 * 1024),
          jobs(JPH::cMaxPhysicsJobs,
               JPH::cMaxPhysicsBarriers,
               std::max(1U, std::thread::hardware_concurrency() > 1U
                                ? std::thread::hardware_concurrency() - 1U
                                : 1U)) {
        physics.Init(kMaximumBodies,
                     0,
                     kMaximumBodyPairs,
                     kMaximumContactConstraints,
                     broadPhaseLayers,
                     objectVsBroadPhase,
                     objectPairs);
        physics.SetGravity(JPH::Vec3::sZero());
    }

    ~Impl() = default;

    static constexpr JPH::uint kMaximumBodies = 65'536;
    static constexpr JPH::uint kMaximumBodyPairs = 65'536;
    static constexpr JPH::uint kMaximumContactConstraints = 20'480;

    JoltRuntime runtime;
    BroadPhaseLayerInterface broadPhaseLayers;
    ObjectVsBroadPhaseFilter objectVsBroadPhase;
    ObjectLayerPairFilter objectPairs;
    JPH::TempAllocatorImpl temporaryAllocator;
    JPH::JobSystemThreadPool jobs;
    JPH::PhysicsSystem physics;
};

PhysicsEngine::PhysicsEngine() : impl_(std::make_unique<Impl>()) {
    stats_.backend = "Jolt Physics 5.6.0 (double precision)";
}

PhysicsEngine::~PhysicsEngine() = default;

void PhysicsEngine::update(float deltaSeconds) {
    constexpr double fixedStep = 1.0 / 60.0;
    constexpr int maximumCatchupSteps = 4;
    accumulatedTime_ += static_cast<double>(std::clamp(deltaSeconds, 0.0F, 0.1F));

    int performedSteps = 0;
    while (accumulatedTime_ >= fixedStep && performedSteps < maximumCatchupSteps) {
        impl_->physics.Update(static_cast<float>(fixedStep), 1, &impl_->temporaryAllocator, &impl_->jobs);
        accumulatedTime_ -= fixedStep;
        ++performedSteps;
        ++stats_.simulationTick;
    }
    if (performedSteps == maximumCatchupSteps) {
        accumulatedTime_ = std::min(accumulatedTime_, fixedStep);
    }

    stats_.bodyCount = impl_->physics.GetNumBodies();
    stats_.activeBodyCount = impl_->physics.GetNumActiveBodies(JPH::EBodyType::RigidBody);
}

} // namespace voxel

#else

namespace voxel {

struct PhysicsEngine::Impl {};
PhysicsEngine::PhysicsEngine() : impl_(std::make_unique<Impl>()) {}
PhysicsEngine::~PhysicsEngine() = default;
void PhysicsEngine::update(float) {}

} // namespace voxel

#endif
