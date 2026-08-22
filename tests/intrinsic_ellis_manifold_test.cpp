#include "app/intrinsic_ellis_camera.hpp"
#include "render/intrinsic_ellis_manifold.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <string>

namespace {

using voxel::EllisFrame;
using voxel::EllisState;
using voxel::EllisTangent;
using voxel::IntrinsicEllisCamera;
using voxel::IntrinsicEllisSettings;
using voxel::PortalVector;

void require(bool condition, const std::string& message) {
    if (!condition) {
        std::cerr << "FAIL: " << message << '\n';
        std::exit(1);
    }
}

float distance(PortalVector a, PortalVector b) {
    return voxel::portalLength(a - b);
}

float tangentDistance(const EllisTangent& a, const EllisTangent& b) {
    return std::abs(a.radial - b.radial) + distance(a.angular, b.angular);
}

EllisFrame radialFrame(PortalVector n) {
    const auto basis = voxel::ellisAngularBasis(n);
    return {{0.0F, basis[0]}, {0.0F, basis[1]}, {-1.0F, {}}};
}

} // namespace

int main() {
    using namespace voxel;
    constexpr float throat = kPortalGrThroatRatio;

    // Pole-safe atlas: the tangent basis remains finite and orthonormal at
    // conventional theta/phi singularities and around their ULP neighborhood.
    for (const PortalVector n : {
             PortalVector{0.0F, 1.0F, 0.0F},
             PortalVector{0.0F, -1.0F, 0.0F},
             portalNormalize({1.0e-7F, 1.0F, -2.0e-7F}),
             portalNormalize({-1.0F, 1.0e-8F, 1.0e-8F})}) {
        const auto basis = ellisAngularBasis(n);
        require(portalFinite(basis[0]) && portalFinite(basis[1]),
                "pole atlas emitted a nonfinite tangent basis");
        require(std::abs(portalDot(n, basis[0])) < 2.0e-5F &&
                std::abs(portalDot(n, basis[1])) < 2.0e-5F,
                "pole atlas basis is not tangent");
        require(std::abs(portalDot(basis[0], basis[1])) < 2.0e-5F,
                "pole atlas basis is not orthogonal");
    }

    // Radial geodesics are analytically l(s)=l0-s. Crossing the throat is C1
    // and cannot rotate a parallel-transported tangent frame.
    EllisState radial{0.62F, {0.0F, 0.0F, 1.0F}, {-1.0F, {}}};
    EllisFrame radialTransport = radialFrame(radial.angularPosition);
    const EllisFrame radialStartFrame = radialTransport;
    ellisIntegrateGeodesic(radial, radialTransport, 0.82F, throat, 256U);
    require(std::abs(radial.properDepth + 0.20F) < 2.0e-4F,
            "radial Ellis geodesic did not cross l=0 analytically");
    require(distance(radial.angularPosition, {0.0F, 0.0F, 1.0F}) < 1.0e-6F,
            "radial throat crossing changed angular position");
    require(tangentDistance(radialTransport.forward,
                            radialStartFrame.forward) < 2.0e-5F &&
            tangentDistance(radialTransport.up, radialStartFrame.up) < 2.0e-5F,
            "radial parallel transport rotated the camera frame");
    require(ellisFrameHandedness(radialTransport,
                                 radial.angularPosition) > 0.999F,
            "radial throat crossing lost positive frame handedness");

    // A nonradial geodesic and its transported frame must be time reversible.
    const auto equator = ellisAngularBasis({0.0F, 0.0F, 1.0F});
    EllisState reversible{0.54F, {0.0F, 0.0F, 1.0F},
                          ellisNormalize({-0.81F, equator[0] * 0.58643F},
                                         {0.0F, 0.0F, 1.0F})};
    EllisFrame reversibleFrame = radialFrame(reversible.angularPosition);
    reversibleFrame.forward = reversible.velocity;
    reversibleFrame.right = ellisNormalize(
        reversibleFrame.right - reversibleFrame.forward *
            ellisDot(reversibleFrame.right, reversibleFrame.forward),
        reversible.angularPosition);
    reversibleFrame.up = ellisNormalize(
        reversibleFrame.up - reversibleFrame.forward *
            ellisDot(reversibleFrame.up, reversibleFrame.forward) -
            reversibleFrame.right *
            ellisDot(reversibleFrame.up, reversibleFrame.right),
        reversible.angularPosition);
    const EllisState reversibleStart = reversible;
    const EllisFrame reversibleFrameStart = reversibleFrame;
    ellisIntegrateGeodesic(reversible, reversibleFrame, 0.43F, throat, 256U);
    ellisIntegrateGeodesic(reversible, reversibleFrame, -0.43F, throat, 256U);
    require(std::abs(reversible.properDepth - reversibleStart.properDepth) <
                3.0e-4F &&
            distance(reversible.angularPosition,
                     reversibleStart.angularPosition) < 4.0e-4F &&
            tangentDistance(reversible.velocity,
                            reversibleStart.velocity) < 8.0e-4F,
            "nonradial intrinsic geodesic failed time reversal");
    require(tangentDistance(reversibleFrame.forward,
                            reversibleFrameStart.forward) < 1.2e-3F &&
            ellisFrameHandedness(reversibleFrame,
                                 reversible.angularPosition) > 0.995F,
            "parallel-transported frame failed time reversal");

    // Compare a normal integration step against a much finer incremental CPU
    // oracle. The same equations are used, but the independent step partition
    // exposes interpolation/accumulation regressions near the throat.
    EllisState coarse{0.71F, portalNormalize({0.3F, 0.91F, -0.28F}), {}};
    const auto coarseBasis = ellisAngularBasis(coarse.angularPosition);
    coarse.velocity = ellisNormalize(
        {-0.70F, coarseBasis[0] * 0.62F + coarseBasis[1] * 0.35F},
        coarse.angularPosition);
    EllisFrame coarseFrame = radialFrame(coarse.angularPosition);
    EllisState oracle = coarse;
    EllisFrame oracleFrame = coarseFrame;
    ellisIntegrateGeodesic(coarse, coarseFrame, 0.88F, throat, 256U);
    constexpr std::uint32_t oracleSteps = 2048U;
    for (std::uint32_t index = 0U; index < oracleSteps; ++index) {
        ellisIntegrateGeodesic(oracle, oracleFrame,
                               0.88F / static_cast<float>(oracleSteps),
                               throat, 4U);
    }
    require(std::abs(coarse.properDepth - oracle.properDepth) < 7.0e-4F &&
            distance(coarse.angularPosition, oracle.angularPosition) <
                1.0e-3F &&
            tangentDistance(coarse.velocity, oracle.velocity) < 1.2e-3F,
            "intrinsic integrator diverged from high-resolution oracle");

    // The free-fly observer crosses l=0 without a chart change or frame snap.
    IntrinsicEllisSettings settings{};
    settings.freeFlySpeed = 0.38F;
    IntrinsicEllisCamera camera;
    camera.reset(settings);
    EllisState nearThroat = camera.state();
    nearThroat.properDepth = 0.012F;
    nearThroat.velocity = {-1.0F, {}};
    camera.setState(nearThroat, camera.frame());
    const EllisFrame before = camera.frame();
    camera.move(1.0F, 0.0F, 0.0F, 0.08F, false, settings);
    require(camera.state().properDepth < 0.0F,
            "intrinsic free-fly did not traverse the throat");
    require(camera.handedness() > 0.995F,
            "intrinsic free-fly frame flipped at the throat");
    require(tangentDistance(before.forward, camera.frame().forward) < 2.0e-3F,
            "radial free-fly camera snapped orientation at l=0");

    // The old lab clamped the observer to 0.999*contentExitProperDepth, which
    // felt like an invisible sphere.  Free flight must now continue through
    // each attachment into both asymptotic content ends.
    EllisState positiveOutside = camera.state();
    positiveOutside.properDepth = settings.contentExitProperDepth * 0.995F;
    positiveOutside.velocity = {1.0F, {}};
    EllisFrame outwardPositive = radialFrame(positiveOutside.angularPosition);
    outwardPositive.forward = {1.0F, {}};
    outwardPositive.right = outwardPositive.right * -1.0F;
    camera.setState(positiveOutside, outwardPositive);
    for (std::uint32_t step = 0U; step < 160U; ++step) {
        camera.move(1.0F, 0.0F, 0.0F, 0.05F, true, settings);
    }
    require(camera.state().properDepth >
                settings.contentExitProperDepth + 2.0F,
            "positive-end free-fly still stops at the content attachment");
    require(camera.motionStatus(settings) ==
                IntrinsicEllisCamera::MotionStatus::PositiveAsymptoticContent &&
            camera.handedness() > 0.995F,
            "positive asymptotic continuation lost status/frame validity");
    const IntrinsicEllisContentPose positivePose = intrinsicEllisContentPose(
        settings, camera.state(), camera.frame(), 192.0F);
    require(positivePose.finite && positivePose.positiveEnd &&
                positivePose.radialDistance >
                    settings.contentSphereRadius * 192.0F,
            "positive content attachment did not extend to the observer");

    EllisState negativeOutside = camera.state();
    negativeOutside.properDepth = -settings.contentExitProperDepth * 0.995F;
    negativeOutside.velocity = {-1.0F, {}};
    EllisFrame outwardNegative = radialFrame(negativeOutside.angularPosition);
    outwardNegative.forward = {-1.0F, {}};
    camera.setState(negativeOutside, outwardNegative);
    for (std::uint32_t step = 0U; step < 160U; ++step) {
        camera.move(1.0F, 0.0F, 0.0F, 0.05F, true, settings);
    }
    require(camera.state().properDepth <
                -settings.contentExitProperDepth - 2.0F,
            "negative-end free-fly still stops at the content attachment");
    require(camera.motionStatus(settings) ==
                IntrinsicEllisCamera::MotionStatus::NegativeAsymptoticContent &&
            std::isfinite(camera.state().properDepth) &&
            portalFinite(camera.state().angularPosition) &&
            camera.handedness() > 0.995F,
            "negative asymptotic continuation is non-finite or flipped");
    const IntrinsicEllisContentPose negativePose = intrinsicEllisContentPose(
        settings, camera.state(), camera.frame(), 192.0F);
    require(negativePose.finite && !negativePose.positiveEnd &&
                negativePose.radialDistance >
                    settings.contentSphereRadius * 192.0F,
            "negative content attachment did not extend to the observer");

    EllisState farFinite = camera.state();
    farFinite.properDepth = -4096.0F;
    const IntrinsicEllisContentPose farPose = intrinsicEllisContentPose(
        settings, farFinite, camera.frame(), 192.0F);
    require(farPose.finite && farPose.radialDistance > 700000.0F,
            "far asymptotic content mapping overflowed or stopped early");

    // The two Ellis ends are glued into one shared playable content chart as
    // a handle. A free-fly observer outside A can therefore enter the visible
    // B mouth (and vice versa), rather than B disappearing from that chart.
    const PortalVector centerA = intrinsicEllisContentCenter(
        settings, true, 1.0F);
    const PortalVector centerB = intrinsicEllisContentCenter(
        settings, false, 1.0F);
    const PortalVector approachNormal = portalNormalize(
        centerB - centerA, {0.0F, 0.0F, 1.0F});
    const auto stateFromAWorld = [&](PortalVector position) {
        const PortalVector relative = position - centerA;
        EllisState result{};
        result.properDepth = settings.contentExitProperDepth +
            portalLength(relative) - settings.contentSphereRadius;
        result.angularPosition = portalNormalize(relative);
        result.velocity = {-1.0F, {}};
        return result;
    };
    const EllisState beforeB = stateFromAWorld(
        centerB - approachNormal * (settings.contentSphereRadius + 0.02F));
    const EllisState afterB = stateFromAWorld(
        centerB - approachNormal * (settings.contentSphereRadius - 0.02F));
    const EllisFrame beforeBFrame = radialFrame(beforeB.angularPosition);
    const EllisFrame afterBFrame = radialFrame(afterB.angularPosition);
    const IntrinsicEllisMouthEntry bEntry = intrinsicEllisOtherMouthEntry(
        settings, beforeB, beforeBFrame, afterB, afterBFrame);
    require(bEntry.entered && !bEntry.positiveEnd &&
                bEntry.segmentFraction >= 0.0F &&
                bEntry.segmentFraction <= 1.0F,
            "shared external chart did not expose/link mouth B from end A");

    const PortalVector approachA = portalNormalize(
        centerA - centerB, {0.0F, 0.0F, 1.0F});
    const auto stateFromBWorld = [&](PortalVector position) {
        const PortalVector relative = position - centerB;
        EllisState result{};
        result.properDepth = -(settings.contentExitProperDepth +
            portalLength(relative) - settings.contentSphereRadius);
        result.angularPosition = portalNormalize(
            intrinsicEllisRotateEndBInverse(
                portalNormalize(relative), settings));
        result.velocity = {1.0F, {}};
        return result;
    };
    const EllisState beforeA = stateFromBWorld(
        centerA - approachA * (settings.contentSphereRadius + 0.02F));
    const EllisState afterA = stateFromBWorld(
        centerA - approachA * (settings.contentSphereRadius - 0.02F));
    const IntrinsicEllisMouthEntry aEntry = intrinsicEllisOtherMouthEntry(
        settings, beforeA, radialFrame(beforeA.angularPosition), afterA,
        radialFrame(afterA.angularPosition));
    require(aEntry.entered && aEntry.positiveEnd,
            "shared external chart did not expose/link mouth A from end B");

    const PortalVector cameraStartWorld = centerB - approachNormal *
        (settings.contentSphereRadius + 0.015F);
    EllisState networkState = stateFromAWorld(cameraStartWorld);
    const PortalVector worldForward = approachNormal;
    PortalVector worldUp = std::abs(portalDot(worldForward,
                                               {0.0F, 1.0F, 0.0F})) < 0.9F
        ? PortalVector{0.0F, 1.0F, 0.0F}
        : PortalVector{0.0F, 0.0F, 1.0F};
    const PortalVector worldRight = portalNormalize(
        portalCross(worldForward, worldUp));
    worldUp = portalNormalize(portalCross(worldRight, worldForward));
    EllisFrame networkFrame{};
    networkFrame.forward = intrinsicEllisTangentFromContent(
        settings, true, networkState.angularPosition, worldForward);
    networkFrame.right = intrinsicEllisTangentFromContent(
        settings, true, networkState.angularPosition, worldRight);
    networkFrame.up = intrinsicEllisTangentFromContent(
        settings, true, networkState.angularPosition, worldUp);
    networkState.velocity = networkFrame.forward;
    camera.setState(networkState, networkFrame);
    camera.move(1.0F, 0.0F, 0.0F, 0.05F, true, settings);
    if (!(camera.lastEnteredMouth() == 1 &&
          camera.state().properDepth < 0.0F &&
          std::abs(camera.state().properDepth) <
              settings.contentExitProperDepth &&
          camera.handedness() > 0.995F)) {
        std::cerr << "network entry diagnostic: last="
                  << camera.lastEnteredMouth() << " l="
                  << camera.state().properDepth << " start="
                  << networkState.properDepth << " handed="
                  << camera.handedness() << '\n';
    }
    require(camera.lastEnteredMouth() == 1 &&
                camera.state().properDepth < 0.0F &&
                std::abs(camera.state().properDepth) <
                    settings.contentExitProperDepth &&
                camera.handedness() > 0.995F,
            "free-fly camera did not enter/transport through visible mouth B");

    const IntrinsicEllisGpuParameters gpu = intrinsicEllisGpuParameters(
        settings, camera.state(), camera.frame());
    for (const auto* value : {&gpu.camera, &gpu.angular, &gpu.forward,
                              &gpu.up, &gpu.endpointA, &gpu.endpointB,
                              &gpu.endBRotation, &gpu.control}) {
        for (float component : *value) {
            require(std::isfinite(component),
                    "intrinsic GPU state contains a nonfinite value");
        }
    }

    // Native signed-depth/view-angle samples agree with a direct high-step
    // oracle away from the deliberately refined critical set. This exercises
    // both asymptotic ends rather than only the old +L mouth entry case.
    const std::filesystem::path cachePath =
        std::filesystem::temp_directory_path() /
        "voxel_intrinsic_ellis_test_v4.bin";
    std::error_code removeError;
    std::filesystem::remove(cachePath, removeError);
    const PortalGrBuildResult tableBuild = buildPortalGrPrecompute(
        cachePath, true);
    require(tableBuild.seconds < 55.0,
            "intrinsic metric precompute exceeded its watchdog budget");
    for (float l : {-0.62F, -0.28F, 0.0F, 0.28F, 0.62F}) {
        for (float q : {0.0F, 0.18F, 0.42F, 0.68F, 0.90F}) {
            const PortalGrGpuSample sampled = samplePortalGrObserverTable(
                tableBuild.table, l, q);
            const PortalGrObserverResult reference =
                integratePortalEllisObserver(l, q, 16384U);
            require(reference.finite && sampled.value[3] < 0.5F,
                    "native observer table produced a nonfinite regular ray");
            require((sampled.value[2] >= 0.0F) == reference.crossed,
                    "native observer table disagrees on the asymptotic end");
            require(std::abs(sampled.value[0] -
                             static_cast<float>(reference.azimuth)) < 0.035F &&
                    std::abs(sampled.value[1] -
                             static_cast<float>(reference.impact)) < 0.006F,
                    "native observer table exceeded its regular-ray oracle bound");
        }
    }

    // Recreate the accepted GPU crossing camera's large circular contour.
    // For an inward local ray the conserved impact is
    // b = sqrt(l^2+a^2) sin(alpha).  b=a is the Ellis unstable throat orbit:
    // the two ULP-near sides genuinely terminate at different asymptotic
    // ends.  This proves that any remaining native-atlas circle follows the
    // physical separatrix rather than the removed Euclidean mouth radius.
    const double nativeObserverL =
        std::sqrt(1.0 - static_cast<double>(throat) * throat) * 0.38;
    const double nativeObserverRadius = std::sqrt(
        nativeObserverL * nativeObserverL +
        static_cast<double>(throat) * throat);
    const double criticalQ = static_cast<double>(throat) /
        nativeObserverRadius;
    const PortalGrObserverResult justInside = integratePortalEllisObserver(
        nativeObserverL, criticalQ - 0.003, 16384U);
    const PortalGrObserverResult justOutside = integratePortalEllisObserver(
        nativeObserverL, criticalQ + 0.003, 16384U);
    require(justInside.finite && justOutside.finite && justInside.crossed &&
                !justOutside.crossed &&
                justInside.impact < static_cast<double>(throat) &&
                justOutside.impact > static_cast<double>(throat),
            "native contour did not coincide with the analytic b=a separatrix");
    std::cout << "Native Ellis separatrix: l=" << nativeObserverL
              << ", alpha=" << std::asin(criticalQ)
              << " rad, b/a sides="
              << justInside.impact / static_cast<double>(throat) << '/'
              << justOutside.impact / static_cast<double>(throat) << '\n';
    std::filesystem::remove(cachePath, removeError);

    std::cout << "Intrinsic Ellis manifold invariants passed: pole-safe atlas, "
                 "C1 throat crossing, time reversal, parallel transport, "
                 "signed-end exit oracle, shared-chart A/B mouths, positive "
                 "frame, finite GPU state.\n";
    return 0;
}
