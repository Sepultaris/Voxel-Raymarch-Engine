#include "app/portal_free_fly_camera.hpp"
#include "app/surface_camera_controller.hpp"
#include "render/portal_lab.hpp"
#include "render/render_settings.hpp"

#include <cmath>
#include <cstring>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>

namespace {

using voxel::PortalLabSettings;
using voxel::PortalVector;

[[noreturn]] void fail(const char* message) {
    std::cerr << "portal lab regression failed: " << message << '\n';
    std::exit(EXIT_FAILURE);
}

void require(bool condition, const char* message) {
    if (!condition) {
        fail(message);
    }
}

bool near(float a, float b, float tolerance = 2.0e-4F) {
    return std::abs(a - b) <= tolerance;
}

bool near(PortalVector a, PortalVector b, float tolerance = 2.0e-4F) {
    return near(a.x, b.x, tolerance) && near(a.y, b.y, tolerance) &&
           near(a.z, b.z, tolerance);
}

} // namespace

int main() {
    PortalLabSettings settings{};
    settings.endpointAElevation = 0.0F;
    settings.endpointADistance = 1.6F;
    settings.endpointBAzimuth = 2.15F;
    settings.endpointBElevation = 0.0F;
    settings.endpointBDistance = 1.6F;
    settings.influenceRadius = 0.32F;
    settings.throatRadius = settings.influenceRadius * voxel::kPortalGrThroatRatio;
    settings.quality = 2U;

    // Cold/warm deterministic GR precompute and cache integrity. The table is
    // the fixed ultrastatic Ellis metric, Christoffel profile, geodesic exit
    // map, and parallel-frame connection consumed by both rays and free-fly.
    const std::filesystem::path grCache =
        std::filesystem::temp_directory_path() / "voxel_portal_gr_test_v4.bin";
    std::error_code cacheError;
    std::filesystem::remove(grCache, cacheError);
    const voxel::PortalGrBuildResult coldGr = voxel::buildPortalGrPrecompute(
        grCache, true);
    require(!coldGr.cacheHit && coldGr.seconds < 55.0,
            "cold GR precompute missed startup watchdog");
    require(coldGr.table.header[0] == voxel::kPortalGrThroatRatio &&
            coldGr.table.status[3] ==
                static_cast<float>(voxel::kPortalGrTableVersion),
            "GR table metric/version header is invalid");
    require(coldGr.table.metric.front().value[0] > 0.0F &&
            coldGr.table.metric.back().value[0] > 0.0F,
            "GR metric table contains a non-positive angular metric");
    const voxel::PortalGrBuildResult warmGr = voxel::buildPortalGrPrecompute(
        grCache, false);
    require(warmGr.cacheHit &&
            std::memcmp(warmGr.table.metric.data(), coldGr.table.metric.data(),
                        sizeof(warmGr.table.metric)) == 0 &&
            std::memcmp(warmGr.table.exit.data(), coldGr.table.exit.data(),
                        sizeof(warmGr.table.exit)) == 0 &&
            std::memcmp(warmGr.table.observer.data(), coldGr.table.observer.data(),
                        sizeof(warmGr.table.observer)) == 0,
            "warm GR cache did not reproduce the cold table");

    // Corrupt/partial cache files are rejected and regenerated rather than
    // uploaded as a valid metric table.
    {
        std::ofstream corrupt(grCache, std::ios::binary | std::ios::trunc);
        corrupt.write("bad", 3);
    }
    const voxel::PortalGrBuildResult repairedGr = voxel::buildPortalGrPrecompute(
        grCache, false);
    require(!repairedGr.cacheHit &&
            repairedGr.table.status[3] ==
                static_cast<float>(voxel::kPortalGrTableVersion),
            "corrupt GR cache was accepted or not regenerated");
    std::filesystem::remove(grCache, cacheError);

    // High-precision oracle checks include radial, ordinary impact, grazing,
    // and both sides of the critical b=a orbit. Intervals certified for direct
    // interpolation must respect the stored error bound; critical intervals
    // are explicitly marked for bounded runtime refinement.
    constexpr std::array<float, 9> grImpacts{
        0.0F, 0.08F, 0.22F, 0.34F, 0.349F, 0.351F, 0.52F, 0.82F, 0.98F};
    for (const float impact : grImpacts) {
        const auto reference = voxel::integratePortalEllisGeodesic(
            impact, 32768U);
        require(reference.finite, "high-precision Ellis oracle became non-finite");
        const auto sampled = voxel::samplePortalGrExitTable(coldGr.table, impact);
        const float bound = std::abs(sampled.value[3]);
        const bool sampledCrossed = sampled.value[3] >= 0.0F;
        if (bound <= 5.0e-4F) {
            require(sampledCrossed == reference.crossed,
                    "certified GR interpolation changed chart outcome");
            require(std::abs(sampled.value[0] - reference.azimuth) <=
                        bound + 2.0e-5F,
                    "GR interpolation exceeded certified azimuth error");
            require(std::abs(sampled.value[1] - reference.affineLength) <=
                        bound + 2.0e-5F,
                    "GR interpolation exceeded certified affine-length error");
            require(std::abs(sampled.value[2] - reference.frameAngle) <=
                        bound + 2.0e-5F,
                    "GR interpolation exceeded certified parallel-frame error");
        } else {
            require(bound > 5.0e-4F,
                    "critical GR interval was not marked for refinement");
        }
    }
    const double observerOuterL = std::sqrt(
        1.0 - static_cast<double>(voxel::kPortalGrThroatRatio) *
                  static_cast<double>(voxel::kPortalGrThroatRatio));
    constexpr std::array<float, 4> observerDepthFractions{
        -0.80F, -0.40F, 0.0F, 0.20F};
    constexpr std::array<float, 4> observerTangents{
        0.0F, 0.20F, 0.48F, 0.72F};
    for (const float depthFraction : observerDepthFractions) {
        for (const float tangent : observerTangents) {
            const float signedL = static_cast<float>(
                observerOuterL * static_cast<double>(depthFraction));
            const auto reference = voxel::integratePortalEllisObserver(
                signedL, tangent, 16384U);
            const auto sampled = voxel::samplePortalGrObserverTable(
                coldGr.table, signedL, tangent);
            require(reference.finite && sampled.value[3] < 0.5F &&
                        (sampled.value[2] >= 0.0F) == reference.crossed,
                    "partial observer exit map changed chart outcome");
            require(std::abs(sampled.value[0] - reference.azimuth) < 0.003F &&
                        std::abs(sampled.value[1] - reference.impact) < 0.001F,
                    "partial observer exit map exceeded interpolation tolerance");
        }
    }

    const PortalVector grOrigin{0.0F, 0.0F, 3.0F};
    const PortalVector grDirection{0.0F, 0.0F, -1.0F};
    const auto grCentral = voxel::tracePortalRayGr(
        grOrigin, grDirection, settings, coldGr.table);
    require(grCentral.affected && grCentral.finite &&
            grCentral.throatTraversals == 1U,
            "central precomputed GR ray did not traverse the Ellis throat");
    for (int y = -14; y <= 14; ++y) {
        for (int x = -14; x <= 14; ++x) {
            const PortalVector rayDirection = voxel::portalNormalize(
                {static_cast<float>(x) * 0.008F,
                 static_cast<float>(y) * 0.008F, -1.0F});
            const auto ray = voxel::tracePortalRayGr(
                grOrigin, rayDirection, settings, coldGr.table);
            require(ray.finite && voxel::portalFinite(ray.origin) &&
                    voxel::portalFinite(ray.direction),
                    "GR portal coverage grid produced a non-finite/gap ray");
        }
    }

    // Disabled is a strict identity apart from direction normalization.
    PortalLabSettings disabled = settings;
    disabled.enabled = false;
    const PortalVector origin{0.0F, 0.0F, 3.0F};
    const PortalVector direction{0.0F, 0.0F, -1.0F};
    const auto identity = voxel::tracePortalRay(origin, direction, disabled);
    require(near(identity.origin, origin, 0.0F), "disabled origin changed");
    require(near(identity.direction, direction, 0.0F), "disabled direction changed");
    require(!identity.affected && identity.integrationSteps == 0U,
            "disabled portal performed work");

    // Rays outside both influence spheres are exact fast-path identities.
    const auto outside = voxel::tracePortalRay(
        {2.9F, 2.9F, 3.0F}, {0.0F, 0.0F, -1.0F}, settings);
    require(!outside.affected && outside.integrationSteps == 0U,
            "outside ray entered the optical integrator");

    // The optical acceleration and its slope go continuously to zero at the
    // influence boundary; no ray kink is introduced there.
    const PortalVector centerA = voxel::portalEndpoint(
        settings.endpointAAzimuth, settings.endpointAElevation,
        settings.endpointADistance);
    const PortalVector tangent{1.0F, 0.0F, 0.0F};
    const PortalVector boundary = centerA + PortalVector{0.0F, 0.0F,
                                                         settings.influenceRadius};
    const auto exactlyBoundary = voxel::portalCurvature(
        boundary, tangent, centerA, settings);
    const auto justInside = voxel::portalCurvature(
        boundary - PortalVector{0.0F, 0.0F, 1.0e-5F}, tangent,
        centerA, settings);
    require(voxel::portalLength(exactlyBoundary) == 0.0F,
            "influence boundary acceleration is nonzero");
    require(voxel::portalLength(justInside) < 1.0e-6F,
            "influence boundary is not C1-like continuous");

    // Mirrored impact parameters bend symmetrically around a spherical mouth.
    PortalLabSettings lensOnly = settings;
    lensOnly.throatRadius = 0.025F;
    lensOnly.endpointBAzimuth = 3.14159265F;
    const auto left = voxel::tracePortalRay(
        {-0.16F, 0.0F, 3.0F}, {0.0F, 0.0F, -1.0F}, lensOnly);
    const auto right = voxel::tracePortalRay(
        {0.16F, 0.0F, 3.0F}, {0.0F, 0.0F, -1.0F}, lensOnly);
    require(left.affected && right.affected, "symmetric rays missed influence sphere");
    require(near(left.direction.x, -right.direction.x, 8.0e-4F) &&
            near(left.direction.y, right.direction.y, 8.0e-4F) &&
            near(left.direction.z, right.direction.z, 8.0e-4F),
            "spherical lens lost radial symmetry");

    // A central ray crosses exactly one linked throat and emerges finite in
    // the destination chart with a normalized direction.
    settings.maximumTraversals = 1U;
    const auto transferred = voxel::tracePortalRay(origin, direction, settings);
    require(transferred.affected, "central ray did not enter portal influence");
    require(transferred.throatTraversals == 1U,
            "central ray did not perform one bounded throat transfer");
    require(transferred.finite && voxel::portalFinite(transferred.origin) &&
            voxel::portalFinite(transferred.direction),
            "throat transfer produced non-finite ray state");
    require(near(voxel::portalLength(transferred.direction), 1.0F, 2.0e-5F),
            "throat transfer direction is not normalized");
    require(transferred.integrationSteps <= 176U,
            "bounded optical integration exceeded its per-ray budget");

    // Medium RK4 must converge to the high-quality reference without the
    // integer-shell discontinuities of the rejected radius/N integrator.
    PortalLabSettings mediumQuality = lensOnly;
    mediumQuality.endpointBAzimuth = 1.5707963F;
    mediumQuality.endpointBDistance = 10.0F;
    mediumQuality.quality = 1U;
    PortalLabSettings highQuality = lensOnly;
    highQuality.endpointBAzimuth = mediumQuality.endpointBAzimuth;
    highQuality.endpointBDistance = mediumQuality.endpointBDistance;
    highQuality.quality = 2U;
    PortalVector previousHighDirection{};
    bool hasPreviousHigh = false;
    for (int sample = 0; sample <= 120; ++sample) {
        const float impact = 0.07F + static_cast<float>(sample) * 0.0015F;
        const PortalVector sampleOrigin{impact, 0.0F, 3.0F};
        const auto mediumRay = voxel::tracePortalRay(
            sampleOrigin, direction, mediumQuality);
        const auto highRay = voxel::tracePortalRay(
            sampleOrigin, direction, highQuality);
        require(mediumRay.finite && highRay.finite,
                "quality-reference ray became non-finite");
        const float qualityAngle = std::acos(std::clamp(
            voxel::portalDot(mediumRay.direction, highRay.direction),
            -1.0F, 1.0F));
        require(qualityAngle < 8.0e-4F,
                "medium optical integration diverged from high reference");
        if (hasPreviousHigh) {
            const float adjacentAngle = std::acos(std::clamp(
                voxel::portalDot(previousHighDirection, highRay.direction),
                -1.0F, 1.0F));
            if (!(adjacentAngle < 0.035F)) {
                std::cerr << "lens continuity diagnostic: impact=" << impact
                          << " adjacent-angle=" << adjacentAngle
                          << " high-steps=" << highRay.integrationSteps
                          << " transfers=" << highRay.throatTraversals << '\n';
            }
            require(adjacentAngle < 0.035F,
                    "adjacent lens samples contain a discontinuous direction band");
        }
        previousHighDirection = highRay.direction;
        hasPreviousHigh = true;
    }

    // Local orientation mapping is isometric: link rotation and endpoint
    // frames preserve direction length for arbitrary portal orientations.
    settings.linkYaw = 0.41F;
    settings.linkPitch = -0.22F;
    settings.linkRoll = 0.17F;
    const auto frameA = voxel::portalFrame(centerA);
    const PortalVector centerB = voxel::portalEndpoint(
        settings.endpointBAzimuth, settings.endpointBElevation,
        settings.endpointBDistance);
    const auto frameB = voxel::portalFrame(centerB);
    const PortalVector arbitrary = voxel::portalNormalize({0.31F, -0.27F, -0.91F});
    const PortalVector local = voxel::portalToLocal(frameA, arbitrary);
    const PortalVector mapped = voxel::portalFromLocal(
        frameB, voxel::portalRotateLink(local, settings));
    require(near(voxel::portalLength(mapped), 1.0F, 2.0e-5F),
            "link transform is not orientation preserving");

    // The linked ray still participates in ordinary exact scene queries. A
    // central destination ray must intersect the conservative planet sphere.
    float planetNear = 0.0F;
    float planetFar = 0.0F;
    const bool destinationReachesPlanet = voxel::portalRaySphere(
        transferred.origin, transferred.direction, {}, 1.10F,
        planetNear, planetFar) && planetFar > 0.0F;
    if (!destinationReachesPlanet) {
        std::cerr << "destination diagnostic origin=(" << transferred.origin.x
                  << ',' << transferred.origin.y << ',' << transferred.origin.z
                  << ") direction=(" << transferred.direction.x << ','
                  << transferred.direction.y << ',' << transferred.direction.z
                  << ") steps=" << transferred.integrationSteps << '\n';
    }
    require(destinationReachesPlanet,
            "destination chart ray cannot reach planet narrow phase");

    // Front-to-back ordering used by the shader is deterministic: an opaque
    // planet shell is closer than a portal placed on its far side.
    float shellNear = 0.0F;
    float shellFar = 0.0F;
    float farPortalNear = 0.0F;
    float farPortalFar = 0.0F;
    require(voxel::portalRaySphere(origin, direction, {}, 1.10F,
                                  shellNear, shellFar),
            "planet ordering reference missed sphere");
    require(voxel::portalRaySphere(origin, direction, {0.0F, 0.0F, -1.6F},
                                  0.32F, farPortalNear, farPortalFar),
            "far portal ordering reference missed sphere");
    require(shellNear < farPortalNear,
            "front terrain should win before a far-side portal");

    // Physical camera/body traversal uses the same linked tangent frames but
    // triggers only when the body center clears the throat rim.
    settings.linkYaw = 0.23F;
    settings.linkPitch = -0.12F;
    settings.linkRoll = 0.08F;
    const PortalVector bodyPrevious{0.0F, 0.0F, 2.0F};
    const PortalVector bodyCurrent{0.0F, 0.0F, 1.50F};
    const PortalVector bodyForward{0.2F, 0.1F, -0.97F};
    const PortalVector bodyVelocity{0.0F, 0.0F, -0.5F};
    const auto bodyTransfer = voxel::portalTransferBodySegment(
        bodyPrevious, bodyCurrent, bodyForward, bodyVelocity, settings, 0.01F);
    require(bodyTransfer.transferred && bodyTransfer.sourceEndpoint == 0U,
            "camera did not transfer A to B on a throat crossing");
    require(voxel::portalFinite(bodyTransfer.position) &&
            near(voxel::portalLength(bodyTransfer.forward), 1.0F, 2.0e-5F),
            "camera transfer pose is invalid");
    const auto sourceBodyFrame = voxel::portalFrame(centerA);
    const auto destinationBodyFrame = voxel::portalFrame(centerB);
    const PortalVector expectedForward = voxel::portalNormalize(
        voxel::portalFromLocal(destinationBodyFrame,
            voxel::portalRotateLink(
                voxel::portalToLocal(sourceBodyFrame,
                                     voxel::portalNormalize(bodyForward)),
                settings)));
    require(near(bodyTransfer.forward, expectedForward, 3.0e-5F),
            "camera orientation was not preserved by the link transform");
    const PortalVector expectedVelocity = voxel::portalFromLocal(
        destinationBodyFrame,
        voxel::portalRotateLink(
            voxel::portalToLocal(sourceBodyFrame, bodyVelocity), settings));
    require(near(bodyTransfer.velocity, expectedVelocity, 3.0e-5F),
            "camera velocity was not preserved by the link transform");

    const auto reverseTransfer = voxel::portalTransferBodySegment(
        {0.0F, 0.0F, 1.35F}, {0.0F, 0.0F, 1.72F},
        bodyForward * -1.0F, bodyVelocity * -1.0F, settings, 0.01F);
    require(reverseTransfer.transferred && reverseTransfer.sourceEndpoint == 0U,
            "back-to-front throat crossing was not detected");

    const auto rimMiss = voxel::portalTransferBodySegment(
        {0.095F, 0.0F, 2.0F}, {0.095F, 0.0F, 1.45F},
        bodyForward, bodyVelocity, settings, 0.02F);
    require(!rimMiss.transferred,
            "capsule touching only the throat rim triggered a transfer");

    const PortalVector afterTransfer = bodyTransfer.position +
        voxel::portalNormalize(bodyTransfer.velocity, bodyTransfer.forward) * 0.02F;
    const auto immediatePingPong = voxel::portalTransferBodySegment(
        bodyTransfer.position, afterTransfer, bodyTransfer.forward,
        bodyTransfer.velocity, settings, 0.01F);
    require(!immediatePingPong.transferred,
            "destination placement immediately ping-ponged through the throat");

    // The linked chart transform is a proper SO(3) map. It is applied once to
    // the observer representation, rather than once in the controller and a
    // second time in the ray shader.
    const PortalVector mappedX = voxel::portalMapVectorAcross(
        {1.0F, 0.0F, 0.0F}, 0U, settings);
    const PortalVector mappedY = voxel::portalMapVectorAcross(
        {0.0F, 1.0F, 0.0F}, 0U, settings);
    const PortalVector mappedZ = voxel::portalMapVectorAcross(
        {0.0F, 0.0F, 1.0F}, 0U, settings);
    require(voxel::portalDot(voxel::portalCross(mappedX, mappedY), mappedZ) >
                0.999F,
            "linked chart transport changed handedness");
    // Replay the user's F7/Home/W center-line crossing. The reference view is
    // exactly what the shader displays: outside the mouth it is the GR ray
    // exit; after the throat it is the persistent physical destination pose
    // continued analytically to that same finite chart boundary.
    // This catches the rejected failure where the controller contributed a
    // progressive link rotation, the shader contributed another full one,
    // and the latter disappeared in a single frame after the throat.
    voxel::PortalManifoldBodyState manifold{};
    PortalVector manifoldPosition = centerA + sourceBodyFrame.z *
        (settings.influenceRadius + 0.008F);
    PortalVector manifoldForward = sourceBodyFrame.z * -1.0F;
    PortalVector manifoldUp = sourceBodyFrame.y;
    const PortalVector initialForward = manifoldForward;
    const PortalVector initialUp = manifoldUp;
    PortalVector manifoldVelocity = manifoldForward * 0.30F;
    float previousProperDepth = settings.influenceRadius - settings.throatRadius;
    bool hasProperDepth = false;
    bool crossedContinuously = false;
    voxel::PortalTraceResult initialVisualRay = voxel::tracePortalObserverRayGr(
        manifoldPosition, manifoldForward, settings, coldGr.table, manifold);
    PortalVector previousVisualPosition = initialVisualRay.affected
        ? initialVisualRay.origin : manifoldPosition;
    PortalVector previousVisualForward = initialVisualRay.affected
        ? initialVisualRay.direction : manifoldForward;
    PortalVector previousVisualUp = voxel::portalMapVectorAcross(
        manifoldUp, 0U, settings);
    float maximumAdjacentViewAngle = 0.0F;
    float maximumAdjacentOriginDelta = 0.0F;
    voxel::PortalManifoldBodyState preCrossObserver{};
    PortalVector preCrossPosition{};
    PortalVector preCrossForward{};
    PortalVector preCrossUp{};
    voxel::PortalManifoldBodyState postCrossObserver{};
    for (int step = 0; step < 240 && !crossedContinuously; ++step) {
        const PortalVector priorPosition = manifoldPosition;
        const PortalVector priorForward = manifoldForward;
        const PortalVector priorUp = manifoldUp;
        const PortalVector activeCenter = manifold.destinationSide
            ? centerB : centerA;
        const PortalVector motion = voxel::portalNormalize(
            activeCenter - manifoldPosition, manifoldForward) * 0.0025F;
        const auto next = voxel::portalAdvanceBodyManifold(
            priorPosition, priorPosition + motion,
            priorForward, priorUp, manifoldVelocity,
            settings, manifold, 0.01F, &coldGr.table);
        require(next.finite, "manifold camera transport became non-finite");
        const voxel::PortalTraceResult visualRay =
            voxel::tracePortalObserverRayGr(
                next.position, next.forward, settings, coldGr.table, next);
        const PortalVector visualPosition = visualRay.affected
            ? visualRay.origin : next.position;
        const PortalVector visualForward = visualRay.affected
            ? visualRay.direction : next.forward;
        const PortalVector visualUp =
            visualRay.affected && !next.destinationSide
                ? voxel::portalMapVectorAcross(next.up, 0U, settings)
                : next.up;
        const float adjacentViewAngle = std::acos(std::clamp(
            voxel::portalDot(voxel::portalNormalize(previousVisualForward),
                             voxel::portalNormalize(visualForward)),
            -1.0F, 1.0F));
        maximumAdjacentViewAngle = std::max(maximumAdjacentViewAngle,
                                             adjacentViewAngle);
        maximumAdjacentOriginDelta = std::max(
            maximumAdjacentOriginDelta,
            voxel::portalLength(visualPosition - previousVisualPosition));
        if (!(adjacentViewAngle < 0.015F)) {
            std::cerr << "observer crossing diagnostic step=" << step
                      << " angle=" << adjacentViewAngle
                      << " progress=" << next.properProgress
                      << " destination=" << next.destinationSide
                      << " crossed=" << next.crossed << '\n';
        }
        require(adjacentViewAngle < 0.015F,
                "rendered camera basis snapped during the full crossing");
        require(voxel::portalLength(visualPosition - previousVisualPosition) <
                    0.012F,
                "rendered camera origin jumped at the chart handoff");
        require(voxel::portalDot(previousVisualUp, visualUp) > 0.999F,
                "rendered camera up snapped during the full crossing");
        if (next.active && !next.destinationSide) {
            require(voxel::portalDot(next.forward, initialForward) > 0.9999F &&
                        voxel::portalDot(next.up, initialUp) > 0.9999F,
                    "source controller basis accumulated the link transform early");
            preCrossObserver = next;
            preCrossPosition = next.position;
            preCrossForward = next.forward;
            preCrossUp = next.up;
        }
        if (next.crossed) {
            postCrossObserver = next;
        }
        const PortalVector right = voxel::portalNormalize(
            voxel::portalCross(visualForward, visualUp));
        require(voxel::portalDot(
                    voxel::portalCross(right, visualForward), visualUp) > 0.999F,
                "transported camera frame lost positive handedness");
        const float properDepth = next.destinationSide
            ? -(voxel::portalLength(next.position - centerB) -
                (settings.throatRadius - 0.01F))
            : voxel::portalLength(next.position - centerA) -
                (settings.throatRadius - 0.01F);
        if (hasProperDepth &&
            !(std::abs(properDepth - previousProperDepth) < 0.008F)) {
            std::cerr << "GR proper-depth diagnostic previous="
                      << previousProperDepth << " next=" << properDepth
                      << " progress=" << next.properProgress
                      << " crossed=" << next.crossed << '\n';
        }
        require(!hasProperDepth ||
                    std::abs(properDepth - previousProperDepth) < 0.008F,
                "intrinsic camera position is not C1 across chart overlap");
        previousProperDepth = properDepth;
        hasProperDepth = next.active;
        manifold = next;
        manifoldPosition = next.position;
        manifoldForward = next.forward;
        manifoldUp = next.up;
        manifoldVelocity = next.velocity;
        previousVisualPosition = visualPosition;
        previousVisualForward = visualForward;
        previousVisualUp = visualUp;
        crossedContinuously = next.crossed;
    }
    require(crossedContinuously,
            "incremental physical traversal never crossed the linked throat");
    const auto radialOracle = voxel::integratePortalEllisGeodesic(0.0, 32768U);
    require(radialOracle.finite && radialOracle.crossed &&
                std::abs(radialOracle.frameAngle) < 2.0e-5,
            "high-precision radial Ellis frame oracle changed unexpectedly");
    const PortalVector expectedFinalForward = voxel::portalNormalize(
        voxel::portalMapVectorAcross(initialForward, 0U, settings));
    const PortalVector expectedFinalUp = voxel::portalOrthonormalUp(
        expectedFinalForward,
        voxel::portalMapVectorAcross(initialUp, 0U, settings));
    require(voxel::portalDot(manifoldForward, expectedFinalForward) > 0.9999F &&
                voxel::portalDot(manifoldUp, expectedFinalUp) > 0.9999F,
            "post-throat observer frame disagrees with high-precision GR reference");
    // Compare an entire pinhole view, not merely its center ray. This is the
    // objective form of the user's video: all off-axis directions immediately
    // before and after the chart event must select the same outgoing GR ray.
    float maximumScreenRayDelta = 0.0F;
    float maximumScreenOriginDelta = 0.0F;
    const PortalVector preRight = voxel::portalNormalize(
        voxel::portalCross(preCrossForward, preCrossUp));
    const PortalVector postRight = voxel::portalNormalize(
        voxel::portalCross(postCrossObserver.forward, postCrossObserver.up));
    for (int y = -3; y <= 3; ++y) {
        for (int x = -4; x <= 4; ++x) {
            const float sx = static_cast<float>(x) * 0.12F;
            const float sy = static_cast<float>(y) * 0.12F;
            const PortalVector beforeDirection = voxel::portalNormalize(
                preCrossForward + preRight * sx + preCrossUp * sy);
            const PortalVector afterDirection = voxel::portalNormalize(
                postCrossObserver.forward + postRight * sx +
                postCrossObserver.up * sy);
            const auto beforeRay = voxel::tracePortalObserverRayGr(
                preCrossPosition, beforeDirection, settings,
                coldGr.table, preCrossObserver);
            const auto afterRay = voxel::tracePortalObserverRayGr(
                postCrossObserver.position, afterDirection, settings,
                coldGr.table, postCrossObserver);
            const float rayAngle = std::acos(std::clamp(
                voxel::portalDot(beforeRay.direction, afterRay.direction),
                -1.0F, 1.0F));
            maximumScreenRayDelta = std::max(maximumScreenRayDelta, rayAngle);
            maximumScreenOriginDelta = std::max(
                maximumScreenOriginDelta,
                voxel::portalLength(beforeRay.origin - afterRay.origin));
        }
    }
    std::cerr << "screen-grid crossing diagnostic max-angle="
              << maximumScreenRayDelta << " max-origin="
              << maximumScreenOriginDelta << '\n';
    require(maximumScreenRayDelta < 0.001F &&
                maximumScreenOriginDelta < 0.001F,
            "off-axis GR view field changed discontinuously at the throat");
    const auto grCameraPolicyRay = voxel::tracePortalRayGr(
        grOrigin, grDirection, settings, coldGr.table);
    require(voxel::portalDot(
                voxel::portalNormalize(manifoldForward),
                voxel::portalNormalize(grCameraPolicyRay.direction)) > 0.995F,
            "free-fly camera and central ray disagree on GR frame transport");
    // Continue well beyond the crossing. The transported frame must remain
    // installed; returning to the original source basis is the exact snap
    // visible in the supplied recording.
    for (int step = 0; step < 48; ++step) {
        const PortalVector priorPosition = manifoldPosition;
        const PortalVector motion = manifoldForward * 0.0025F;
        const auto next = voxel::portalAdvanceBodyManifold(
            priorPosition, priorPosition + motion,
            manifoldForward, manifoldUp, manifoldVelocity,
            settings, manifold, 0.01F, &coldGr.table);
        require(next.finite &&
                    voxel::portalDot(next.forward, expectedFinalForward) > 0.9999F &&
                    voxel::portalDot(next.up, expectedFinalUp) > 0.9999F,
                "post-exit camera returned to its pre-portal basis");
        manifold = next;
        manifoldPosition = next.position;
        manifoldForward = next.forward;
        manifoldUp = next.up;
        manifoldVelocity = next.velocity;
    }

    // The portal-only free-fly controller is true six-DOF motion. It has no
    // terrain query/collision path and retains a positive orthonormal frame.
    voxel::PortalFreeFlyCamera freeFly{};
    freeFly.resetForPortal(settings, 1.0F);
    const PortalVector freeStart = freeFly.position();
    const PortalVector freeForward = freeFly.forward();
    freeFly.move(1.0F, 0.0F, 0.0F, 0.1F, false);
    require(voxel::portalDot(freeFly.position() - freeStart, freeForward) > 0.029F,
            "free-fly forward input did not move along camera forward");
    const PortalVector beforeVertical = freeFly.position();
    const PortalVector freeUp = freeFly.up();
    freeFly.move(0.0F, 0.0F, 1.0F, 0.1F, false);
    require(voxel::portalDot(freeFly.position() - beforeVertical, freeUp) > 0.029F,
            "free-fly vertical input did not move along camera up");
    freeFly.setPose({0.0F, 0.0F, 0.25F}, {0.0F, 0.0F, -1.0F},
                    {0.0F, 1.0F, 0.0F});
    freeFly.move(1.0F, 0.0F, 0.0F, 0.1F, true);
    require(freeFly.position().z < 0.25F,
            "free-fly was incorrectly clamped by planet terrain/collision");
    freeFly.rotate(0.15F, -0.11F, 0.08F);
    const PortalVector freeRight = voxel::portalNormalize(
        voxel::portalCross(freeFly.forward(), freeFly.up()));
    require(voxel::portalDot(
                voxel::portalCross(freeRight, freeFly.forward()),
                freeFly.up()) > 0.999F,
            "free-fly mouse/roll rotation lost positive handedness");

    // A free-fly pose accepts the exact manifold result without projecting it
    // onto radial gravity or the surface collision controller.
    freeFly.setPose(manifold.position, manifold.forward, manifold.up,
                    manifold.velocity);
    require(near(freeFly.position(), manifold.position, 2.0e-5F) &&
            near(freeFly.forward(), manifold.forward, 2.0e-5F) &&
            near(freeFly.up(), manifold.up, 2.0e-5F),
            "free-fly pose did not preserve manifold transport output");
    require(static_cast<std::uint32_t>(voxel::CameraControllerMode::Orbit) !=
                static_cast<std::uint32_t>(voxel::CameraControllerMode::PortalFreeFly) &&
            static_cast<std::uint32_t>(voxel::CameraControllerMode::SurfaceTraversal) !=
                static_cast<std::uint32_t>(voxel::CameraControllerMode::PortalFreeFly),
            "portal free-fly aliases an existing controller mode");
    voxel::SurfaceCameraController independentSurface{};
    const auto surfaceRadialBefore = independentSurface.radial();
    const PortalVector independentFlyBefore = freeFly.position();
    freeFly.move(0.0F, 1.0F, 0.0F, 0.1F, false);
    require(independentSurface.radial() == surfaceRadialBefore &&
            !near(freeFly.position(), independentFlyBefore, 1.0e-6F),
            "free-fly input mutated or aliased the surface controller state");

    PortalLabSettings gpuPose = settings;
    gpuPose.freeFlyPosition = freeFly.position();
    gpuPose.freeFlyForward = freeFly.forward();
    gpuPose.freeFlyUp = freeFly.up();
    gpuPose.observerManifoldActive = true;
    gpuPose.observerDestinationSide = true;
    const auto packedPose = voxel::portalGpuParameters(gpuPose);
    require(near(packedPose.freeFlyPosition[0], freeFly.position().x) &&
            near(packedPose.freeFlyForward[2], freeFly.forward().z) &&
            near(packedPose.freeFlyUp[1], freeFly.up().y),
            "free-fly render pose packing changed the controller frame");
    require(packedPose.control[2] == 1.0F && packedPose.control[3] == 1.0F,
            "destination observer state was not packed for GR continuation");

    // A broad deterministic stress set must stay finite and within the fixed
    // integration/traversal limits, including grazing rays.
    for (int y = -12; y <= 12; ++y) {
        for (int x = -12; x <= 12; ++x) {
            const PortalVector stressDirection = voxel::portalNormalize(
                {static_cast<float>(x) * 0.012F,
                 static_cast<float>(y) * 0.012F, -1.0F});
            const auto ray = voxel::tracePortalRayGr(
                origin, stressDirection, settings, coldGr.table);
            require(ray.finite && voxel::portalFinite(ray.origin) &&
                    voxel::portalFinite(ray.direction),
                    "stress ray became non-finite");
            require(ray.throatTraversals <= 1U,
                    "stress ray exceeded throat traversal limit");
            require(ray.integrationSteps <= 16384U,
                    "stress ray exceeded total integration guard");
        }
    }

    std::cout << "portal lab invariants passed: identity, boundary continuity, "
                 "radial symmetry, smooth linked charts, handedness, shared "
                 "ray/body transport, six-DOF free-fly, mode isolation, "
                 "ordering, finite bounded stress; GR cold="
              << coldGr.seconds * 1000.0 << " ms warm="
              << warmGr.seconds * 1000.0 << " ms direct-error="
              << coldGr.maximumError << " max-view-delta="
              << maximumAdjacentViewAngle << " rad max-origin-delta="
              << maximumAdjacentOriginDelta << '\n';
    return EXIT_SUCCESS;
}
