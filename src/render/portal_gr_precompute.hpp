#pragma once

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <limits>
#include <string>

namespace voxel {

constexpr std::uint32_t kPortalGrMetricSamples = 128U;
constexpr std::uint32_t kPortalGrExitSamples = 256U;
constexpr std::uint32_t kPortalGrObserverDepthSamples = 96U;
constexpr std::uint32_t kPortalGrObserverViewSamples = 96U;
constexpr float kPortalGrThroatRatio = 0.35F;
constexpr std::uint32_t kPortalGrTableVersion = 4U;

struct alignas(16) PortalGrGpuSample {
    std::array<float, 4> value{};
};

struct alignas(16) PortalGrPrecomputedTable {
    // throat/influence, unit influence radius, maximum sampled impact,
    // exit-sample count.
    std::array<float, 4> header{};
    // certified max interpolation error, build seconds, cache hit, version.
    std::array<float, 4> status{};
    std::array<PortalGrGpuSample, kPortalGrMetricSamples> metric{};
    std::array<PortalGrGpuSample, kPortalGrExitSamples> exit{};
    // Signed proper-depth x local tangent-speed exit map for observers already
    // inside either mouth half. value = phi, conserved b, crossed flag, finite.
    std::array<PortalGrGpuSample,
               kPortalGrObserverDepthSamples * kPortalGrObserverViewSamples>
        observer{};
};

struct PortalGrGeodesicResult {
    double azimuth{};
    double affineLength{};
    double frameAngle{};
    bool crossed{};
    bool finite{true};
};

struct PortalGrBuildResult {
    PortalGrPrecomputedTable table{};
    double seconds{};
    float maximumError{};
    bool cacheHit{};
    std::filesystem::path cachePath{};
};

struct PortalGrObserverResult {
    double azimuth{};
    double impact{};
    bool crossed{};
    bool finite{true};
};

// Ultrastatic Ellis wormhole:
// ds^2 = -dt^2 + dl^2 + (l^2 + a^2)(dtheta^2 + sin^2(theta)dphi^2).
// Spherical symmetry permits every geodesic to be integrated in an equatorial
// plane. These are the exact coordinate geodesic equations and the SO(2)
// orthonormal-frame connection along that plane.
[[nodiscard]] inline PortalGrGeodesicResult integratePortalEllisGeodesic(
    double impact, std::uint32_t stepsPerRadius) noexcept {
    constexpr double a = static_cast<double>(kPortalGrThroatRatio);
    constexpr double outer = 1.0;
    const double maximumImpact = outer * 0.999;
    const double b = std::clamp(impact, 0.0, maximumImpact);
    const double L = std::sqrt(outer * outer - a * a);
    struct State {
        double l{};
        double phi{};
        double vl{};
        double omega{};
        double frame{};
    };
    State state{L, 0.0, -std::sqrt(std::max(1.0 - b * b, 0.0)), b, 0.0};
    const double step = 1.0 / static_cast<double>(
        std::max(stepsPerRadius, 256U));
    const std::uint32_t maximumSteps = std::max(stepsPerRadius * 32U, 8192U);
    bool passedTurningRegion = false;
    double length = 0.0;
    const auto derivative = [a](const State& value) noexcept {
        const double r2 = value.l * value.l + a * a;
        const double r = std::sqrt(r2);
        return State{
            value.vl,
            value.omega,
            value.l * value.omega * value.omega,
            -2.0 * value.l / r2 * value.vl * value.omega,
            -(value.l / r) * value.omega};
    };
    const auto add = [](const State& left, const State& right,
                        double scale) noexcept {
        return State{left.l + right.l * scale,
                     left.phi + right.phi * scale,
                     left.vl + right.vl * scale,
                     left.omega + right.omega * scale,
                     left.frame + right.frame * scale};
    };
    for (std::uint32_t iteration = 0U; iteration < maximumSteps; ++iteration) {
        const State previous = state;
        const State k1 = derivative(state);
        const State k2 = derivative(add(state, k1, step * 0.5));
        const State k3 = derivative(add(state, k2, step * 0.5));
        const State k4 = derivative(add(state, k3, step));
        state.l += step * (k1.l + 2.0 * k2.l + 2.0 * k3.l + k4.l) / 6.0;
        state.phi += step * (k1.phi + 2.0 * k2.phi + 2.0 * k3.phi + k4.phi) / 6.0;
        state.vl += step * (k1.vl + 2.0 * k2.vl + 2.0 * k3.vl + k4.vl) / 6.0;
        state.omega += step * (k1.omega + 2.0 * k2.omega +
                               2.0 * k3.omega + k4.omega) / 6.0;
        state.frame += step * (k1.frame + 2.0 * k2.frame +
                               2.0 * k3.frame + k4.frame) / 6.0;
        length += step;
        if (!std::isfinite(state.l) || !std::isfinite(state.phi) ||
            !std::isfinite(state.vl) || !std::isfinite(state.omega) ||
            !std::isfinite(state.frame)) {
            return {{}, {}, {}, false, false};
        }
        passedTurningRegion = passedTurningRegion || state.vl > 0.0 || state.l < 0.0;
        const bool crossed = state.l <= -L;
        const bool scattered = passedTurningRegion && state.vl > 0.0 && state.l >= L;
        if (crossed || scattered) {
            const double boundary = crossed ? -L : L;
            const double denominator = state.l - previous.l;
            const double fraction = std::abs(denominator) > 1.0e-14
                ? std::clamp((boundary - previous.l) / denominator, 0.0, 1.0)
                : 1.0;
            return {
                previous.phi + (state.phi - previous.phi) * fraction,
                length - step + step * fraction,
                previous.frame + (state.frame - previous.frame) * fraction,
                crossed,
                true};
        }
    }
    return {state.phi, length, state.frame, false, false};
}

[[nodiscard]] inline PortalGrObserverResult integratePortalEllisObserver(
    double signedProperL, double tangentSpeed,
    std::uint32_t stepsPerRadius) noexcept {
    constexpr double a = static_cast<double>(kPortalGrThroatRatio);
    const double outerL = std::sqrt(1.0 - a * a);
    struct State { double l, phi, vl, omega; };
    const double l0 = std::clamp(signedProperL, -outerL, outerL);
    const double q = std::clamp(tangentSpeed, 0.0, 0.985);
    const double r0 = std::sqrt(l0 * l0 + a * a);
    const double impact = r0 * q;
    State state{l0, 0.0, -std::sqrt(std::max(1.0 - q * q, 0.0)),
                impact / (r0 * r0)};
    if (l0 <= -outerL + 1.0e-12) {
        return {0.0, impact, true, true};
    }
    const double step = 1.0 / static_cast<double>(
        std::max(stepsPerRadius, 512U));
    const std::uint32_t maximumSteps = std::max(stepsPerRadius * 12U, 8192U);
    bool turned = false;
    const auto derivative = [a](const State& value) noexcept {
        const double radius2 = value.l * value.l + a * a;
        return State{value.vl, value.omega,
                     value.l * value.omega * value.omega,
                     -2.0 * value.l / radius2 * value.vl * value.omega};
    };
    const auto add = [](State left, State right, double scale) noexcept {
        return State{left.l + right.l * scale,
                     left.phi + right.phi * scale,
                     left.vl + right.vl * scale,
                     left.omega + right.omega * scale};
    };
    for (std::uint32_t iteration = 0U; iteration < maximumSteps; ++iteration) {
        const State previous = state;
        const State k1 = derivative(state);
        const State k2 = derivative(add(state, k1, step * 0.5));
        const State k3 = derivative(add(state, k2, step * 0.5));
        const State k4 = derivative(add(state, k3, step));
        state.l += step * (k1.l + 2.0 * k2.l + 2.0 * k3.l + k4.l) / 6.0;
        state.phi += step * (k1.phi + 2.0 * k2.phi + 2.0 * k3.phi + k4.phi) / 6.0;
        state.vl += step * (k1.vl + 2.0 * k2.vl + 2.0 * k3.vl + k4.vl) / 6.0;
        state.omega += step * (k1.omega + 2.0 * k2.omega +
                               2.0 * k3.omega + k4.omega) / 6.0;
        if (!std::isfinite(state.l) || !std::isfinite(state.phi) ||
            !std::isfinite(state.vl) || !std::isfinite(state.omega)) {
            return {state.phi, impact, false, false};
        }
        turned = turned || state.vl > 0.0;
        const bool crossed = state.l <= -outerL;
        const bool scattered = turned && state.vl > 0.0 && state.l >= outerL;
        if (crossed || scattered) {
            const double boundary = crossed ? -outerL : outerL;
            const double denominator = state.l - previous.l;
            const double fraction = std::abs(denominator) > 1.0e-14
                ? std::clamp((boundary - previous.l) / denominator, 0.0, 1.0)
                : 1.0;
            return {previous.phi + (state.phi - previous.phi) * fraction,
                    impact, crossed, true};
        }
    }
    return {state.phi, impact, false, false};
}

[[nodiscard]] inline std::uint64_t portalGrChecksum(
    const PortalGrPrecomputedTable& table) noexcept {
    const auto* bytes = reinterpret_cast<const unsigned char*>(&table);
    std::uint64_t hash = 1469598103934665603ULL;
    for (std::size_t index = 0U; index < sizeof(table); ++index) {
        hash = (hash ^ bytes[index]) * 1099511628211ULL;
    }
    return hash;
}

struct PortalGrCacheFile {
    std::array<char, 8> magic{'V', 'X', 'G', 'R', 'E', 'L', 'L', '4'};
    std::uint32_t version{kPortalGrTableVersion};
    std::uint32_t bytes{static_cast<std::uint32_t>(sizeof(PortalGrPrecomputedTable))};
    std::uint64_t checksum{};
    PortalGrPrecomputedTable table{};
};

[[nodiscard]] inline PortalGrBuildResult buildPortalGrPrecompute(
    const std::filesystem::path& cachePath,
    bool forceRegeneration = false) {
    PortalGrBuildResult result{};
    result.cachePath = cachePath;
    const auto started = std::chrono::steady_clock::now();
    if (!forceRegeneration) {
        std::ifstream input(cachePath, std::ios::binary);
        PortalGrCacheFile cache{};
        if (input.read(reinterpret_cast<char*>(&cache), sizeof(cache)) &&
            cache.magic == PortalGrCacheFile{}.magic &&
            cache.version == kPortalGrTableVersion &&
            cache.bytes == sizeof(PortalGrPrecomputedTable) &&
            cache.checksum == portalGrChecksum(cache.table)) {
            result.table = cache.table;
            result.maximumError = cache.table.status[0];
            result.cacheHit = true;
            result.seconds = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - started).count();
            result.table.status[1] = static_cast<float>(result.seconds);
            result.table.status[2] = 1.0F;
            return result;
        }
    }

    constexpr double a = static_cast<double>(kPortalGrThroatRatio);
    const double L = std::sqrt(1.0 - a * a);
    result.table.header = {kPortalGrThroatRatio, 1.0F, 0.999F,
                           static_cast<float>(kPortalGrExitSamples)};
    for (std::uint32_t index = 0U; index < kPortalGrMetricSamples; ++index) {
        const double fraction = static_cast<double>(index) /
            static_cast<double>(kPortalGrMetricSamples - 1U);
        const double l = L * (1.0 - 2.0 * fraction);
        const double angularMetric = l * l + a * a;
        result.table.metric[index].value = {
            static_cast<float>(angularMetric), static_cast<float>(-l),
            static_cast<float>(l / angularMetric),
            static_cast<float>(l / std::sqrt(angularMetric))};
    }
    for (std::uint32_t depth = 0U; depth < kPortalGrObserverDepthSamples;
         ++depth) {
        const double depthFraction = static_cast<double>(depth) /
            static_cast<double>(kPortalGrObserverDepthSamples - 1U);
        const double signedL = L * (1.0 - 2.0 * depthFraction);
        for (std::uint32_t view = 0U; view < kPortalGrObserverViewSamples;
             ++view) {
            const double viewFraction = static_cast<double>(view) /
                static_cast<double>(kPortalGrObserverViewSamples - 1U);
            const double tangentSpeed = viewFraction * 0.985;
            const PortalGrObserverResult observer = integratePortalEllisObserver(
                signedL, tangentSpeed, 4096U);
            result.table.observer[
                depth * kPortalGrObserverViewSamples + view].value = {
                    static_cast<float>(observer.azimuth),
                    static_cast<float>(observer.impact),
                    observer.crossed ? 1.0F : -1.0F,
                    observer.finite ? 0.0F : 1.0F};
        }
    }
    float maximumError = 0.0F;
    for (std::uint32_t index = 0U; index < kPortalGrExitSamples; ++index) {
        const double fraction = static_cast<double>(index) /
            static_cast<double>(kPortalGrExitSamples - 1U);
        const double impact = fraction * 0.999;
        const PortalGrGeodesicResult coarse = integratePortalEllisGeodesic(
            impact, 4096U);
        const PortalGrGeodesicResult reference = integratePortalEllisGeodesic(
            impact, 16384U);
        float error = std::numeric_limits<float>::infinity();
        if (coarse.finite && reference.finite &&
            coarse.crossed == reference.crossed) {
            error = static_cast<float>(std::max({
                std::abs(coarse.azimuth - reference.azimuth),
                std::abs(coarse.frameAngle - reference.frameAngle),
                std::abs(coarse.affineLength - reference.affineLength)}));
        }
        if (!std::isfinite(error)) {
            error = 1000.0F;
        }
        maximumError = std::max(maximumError,
            error < 1.0F ? error : 0.0F);
        result.table.exit[index].value = {
            static_cast<float>(reference.azimuth),
            static_cast<float>(reference.affineLength),
            static_cast<float>(reference.frameAngle),
            reference.finite ? (reference.crossed ? error : -error) : -1000.0F};
    }
    // Certify interpolation, not merely the integration at stored nodes. Any
    // interval whose midpoint error exceeds the runtime tolerance is marked
    // for bounded local refinement at both endpoints.
    for (std::uint32_t index = 0U; index + 1U < kPortalGrExitSamples; ++index) {
        const double midpointFraction =
            (static_cast<double>(index) + 0.5) /
            static_cast<double>(kPortalGrExitSamples - 1U);
        const PortalGrGeodesicResult reference = integratePortalEllisGeodesic(
            midpointFraction * 0.999, 16384U);
        auto& left = result.table.exit[index].value;
        auto& right = result.table.exit[index + 1U].value;
        const bool leftCrossed = left[3] >= 0.0F;
        const bool rightCrossed = right[3] >= 0.0F;
        if (!reference.finite || leftCrossed != rightCrossed ||
            leftCrossed != reference.crossed) {
            left[3] = leftCrossed ? 1000.0F : -1000.0F;
            right[3] = rightCrossed ? 1000.0F : -1000.0F;
            continue;
        }
        const float interpolationError = static_cast<float>(std::max({
            std::abs(reference.azimuth -
                0.5 * static_cast<double>(left[0] + right[0])),
            std::abs(reference.affineLength -
                0.5 * static_cast<double>(left[1] + right[1])),
            std::abs(reference.frameAngle -
                0.5 * static_cast<double>(left[2] + right[2]))}));
        const auto preserveSign = [](float value, float magnitude) noexcept {
            return value >= 0.0F ? magnitude : -magnitude;
        };
        left[3] = preserveSign(left[3], std::max(std::abs(left[3]),
                                                 interpolationError));
        right[3] = preserveSign(right[3], std::max(std::abs(right[3]),
                                                   interpolationError));
        if (interpolationError < 1.0F) {
            maximumError = std::max(maximumError, interpolationError);
        }
    }
    maximumError = 0.0F;
    for (const PortalGrGpuSample& sample : result.table.exit) {
        const float error = std::abs(sample.value[3]);
        if (error <= 5.0e-4F) {
            maximumError = std::max(maximumError, error);
        }
    }
    result.maximumError = maximumError;
    result.seconds = std::chrono::duration<double>(
        std::chrono::steady_clock::now() - started).count();
    result.table.status = {maximumError, static_cast<float>(result.seconds),
                           0.0F, static_cast<float>(kPortalGrTableVersion)};

    PortalGrCacheFile cache{};
    cache.table = result.table;
    cache.checksum = portalGrChecksum(cache.table);
    std::error_code errorCode;
    std::filesystem::create_directories(cachePath.parent_path(), errorCode);
    const std::filesystem::path temporary = cachePath.string() + ".partial";
    {
        std::ofstream output(temporary, std::ios::binary | std::ios::trunc);
        if (output) {
            output.write(reinterpret_cast<const char*>(&cache), sizeof(cache));
        }
    }
    std::filesystem::remove(cachePath, errorCode);
    errorCode.clear();
    std::filesystem::rename(temporary, cachePath, errorCode);
    if (errorCode) {
        std::filesystem::remove(temporary, errorCode);
    }
    return result;
}

[[nodiscard]] inline PortalGrGpuSample samplePortalGrExitTable(
    const PortalGrPrecomputedTable& table, float normalizedImpact) noexcept {
    const float coordinate = std::clamp(normalizedImpact /
        std::max(table.header[2], 1.0e-6F), 0.0F, 1.0F) *
        static_cast<float>(kPortalGrExitSamples - 1U);
    const std::uint32_t low = static_cast<std::uint32_t>(coordinate);
    const std::uint32_t high = std::min(low + 1U, kPortalGrExitSamples - 1U);
    const float fraction = coordinate - static_cast<float>(low);
    PortalGrGpuSample result{};
    for (std::size_t component = 0U; component < 4U; ++component) {
        result.value[component] = table.exit[low].value[component] * (1.0F - fraction) +
                                  table.exit[high].value[component] * fraction;
    }
    return result;
}

[[nodiscard]] inline PortalGrGpuSample samplePortalGrObserverTable(
    const PortalGrPrecomputedTable& table, float signedProperL,
    float tangentSpeed) noexcept {
    const float outerL = std::sqrt(std::max(
        1.0F - kPortalGrThroatRatio * kPortalGrThroatRatio, 1.0e-8F));
    const float depthCoordinate = std::clamp(
        (outerL - signedProperL) / (2.0F * outerL), 0.0F, 1.0F) *
        static_cast<float>(kPortalGrObserverDepthSamples - 1U);
    const float viewCoordinate = std::clamp(tangentSpeed / 0.985F,
                                             0.0F, 1.0F) *
        static_cast<float>(kPortalGrObserverViewSamples - 1U);
    const std::uint32_t d0 = static_cast<std::uint32_t>(depthCoordinate);
    const std::uint32_t d1 = std::min(d0 + 1U,
                                      kPortalGrObserverDepthSamples - 1U);
    const std::uint32_t v0 = static_cast<std::uint32_t>(viewCoordinate);
    const std::uint32_t v1 = std::min(v0 + 1U,
                                      kPortalGrObserverViewSamples - 1U);
    const float fd = depthCoordinate - static_cast<float>(d0);
    const float fv = viewCoordinate - static_cast<float>(v0);
    const auto& a = table.observer[d0 * kPortalGrObserverViewSamples + v0].value;
    const auto& b = table.observer[d0 * kPortalGrObserverViewSamples + v1].value;
    const auto& c = table.observer[d1 * kPortalGrObserverViewSamples + v0].value;
    const auto& d = table.observer[d1 * kPortalGrObserverViewSamples + v1].value;
    PortalGrGpuSample result{};
    for (std::size_t component = 0U; component < 4U; ++component) {
        const float upper = a[component] * (1.0F - fv) + b[component] * fv;
        const float lower = c[component] * (1.0F - fv) + d[component] * fv;
        result.value[component] = upper * (1.0F - fd) + lower * fd;
    }
    return result;
}

} // namespace voxel
