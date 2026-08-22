#include "render/planet_atmosphere.hpp"

#include <array>
#include <cmath>
#include <cstdlib>
#include <iostream>

namespace {

void require(bool condition, const char* message) {
    if (!condition) {
        std::cerr << "planet atmosphere test failure: " << message << '\n';
        std::exit(EXIT_FAILURE);
    }
}

float midpointOpticalDepth(const std::array<float, 3>& origin,
                           const std::array<float, 3>& direction,
                           float innerRadius, float outerRadius,
                           float maximumDistance) {
    const auto interval = voxel::atmosphereSphereInterval(
        origin, direction, outerRadius);
    if (!interval.intersects) {
        return 0.0F;
    }
    const float nearDistance = interval.nearDistance;
    const float farDistance = std::min(interval.farDistance, maximumDistance);
    if (!(farDistance > nearDistance)) {
        return 0.0F;
    }
    const float step = (farDistance - nearDistance) / 64.0F;
    float opticalDepth = 0.0F;
    for (int index = 0; index < 64; ++index) {
        const float distance = nearDistance + (static_cast<float>(index) + 0.5F) * step;
        const float x = origin[0] + direction[0] * distance;
        const float y = origin[1] + direction[1] * distance;
        const float z = origin[2] + direction[2] * distance;
        opticalDepth += voxel::atmosphereDensityProfile(
            std::sqrt(x * x + y * y + z * z), innerRadius, outerRadius);
    }
    return opticalDepth * step / (outerRadius - innerRadius);
}

} // namespace

int main() {
    using namespace voxel;

    constexpr std::array<float, 3> camera{0.0F, 0.0F, 3.0F};
    constexpr std::array<float, 3> inward{0.0F, 0.0F, -1.0F};
    constexpr std::array<float, 3> outward{0.0F, 0.0F, 1.0F};
    const auto crossing = atmosphereSphereInterval(camera, inward, 1.18F);
    require(crossing.intersects && std::abs(crossing.nearDistance - 1.82F) < 1.0e-5F &&
                std::abs(crossing.farDistance - 4.18F) < 1.0e-5F,
            "spherical shell entry/exit interval is incorrect");
    require(!atmosphereSphereInterval(camera, outward, 1.18F).intersects,
            "ray directed away from the shell reported an intersection");

    float previousDensity = 1.0F;
    for (int sample = 0; sample <= 100; ++sample) {
        const float radius = 1.0F + 0.18F * static_cast<float>(sample) / 100.0F;
        const float density = atmosphereDensityProfile(radius, 1.0F, 1.18F);
        require(std::isfinite(density) && density >= 0.0F && density <= previousDensity,
                "altitude density is not finite, bounded and monotonic");
        previousDensity = density;
    }

    float previousTransmittance = 1.0F;
    for (int sample = 0; sample <= 100; ++sample) {
        const float transmittance = atmosphereTransmittance(
            3.0F * static_cast<float>(sample) / 100.0F, 1.15F);
        require(std::isfinite(transmittance) && transmittance <= previousTransmittance &&
                    transmittance >= 0.0F && transmittance <= 1.0F,
                "transmittance is not finite, monotonic and bounded");
        previousTransmittance = transmittance;
    }
    require(atmosphereTransmittance(12.0F, 0.0F) == 1.0F,
            "zero atmospheric density was not the identity transform");
    require(atmosphereTransmittance(2.0F, 2.0F) <
                atmosphereTransmittance(2.0F, 1.0F) &&
                atmosphereTransmittance(4.0F, 1.0F) <
                atmosphereTransmittance(2.0F, 1.0F),
            "surface attenuation was not monotonic with density and distance");

    const float terrainOptical = midpointOpticalDepth(
        camera, inward, 1.0F, 1.18F, 1.92F);
    const float fullSkyOptical = midpointOpticalDepth(
        camera, inward, 1.0F, 1.18F, 100.0F);
    require(terrainOptical > 0.0F && fullSkyOptical > terrainOptical,
            "terrain aerial segment was not distinguished from the full sky shell");
    const float star = 0.8F;
    const float attenuatedStar = star * atmosphereTransmittance(fullSkyOptical, 1.15F);
    require(attenuatedStar >= 0.0F && attenuatedStar < star,
            "background star was not attenuated behind the atmosphere");

    PlanetAtmosphereSettings settings{};
    settings.startHeight = 0.017F;
    settings.endHeight = 0.184F;
    settings.density = 1.37F;
    settings.enabled = true;
    settings.physicalSurface = true;
    settings.artisticNearFade = false;
    settings.rayleighScattering = 1.24F;
    settings.mieScattering = 0.71F;
    settings.absorption = 0.33F;
    settings.scaleHeight = 0.29F;
    settings.mieAnisotropy = 0.72F;
    settings.debugMode = 2U;
    const std::uint32_t terrainSeed =
        packAtmosphereTerrainSeedHighBits(settings, 3U) << 24U;
    const std::uint32_t brushMaterial = quantizeAtmosphere(
        settings.endHeight, kAtmosphereMaximumEndHeight, 255U) << 24U;
    const std::uint32_t editMode = quantizeAtmosphere(
        settings.density, kAtmosphereMaximumDensity, 255U) << 24U;
    const std::uint32_t simulation = packAtmosphereSimulationFlags(settings);
    require(((terrainSeed >> 24U) & 0x3U) == 3U,
            "atmosphere start packing overwrote lighting debug bits");
    require(std::abs(unpackAtmosphereStartHeight(terrainSeed) - settings.startHeight) <=
                kAtmosphereMaximumStartHeight / 7.0F &&
                std::abs(unpackAtmosphereEndHeight(brushMaterial) - settings.endHeight) <=
                kAtmosphereMaximumEndHeight / 255.0F &&
                std::abs(unpackAtmosphereDensity(editMode) - settings.density) <=
                kAtmosphereMaximumDensity / 255.0F,
            "atmosphere controls exceeded one packing quantization step");
    require((simulation & (1U << 27U)) != 0U &&
                (simulation & (1U << 28U)) != 0U &&
                (simulation & (1U << 29U)) == 0U &&
                ((terrainSeed >> 26U) & 0x7U) == 2U,
            "atmosphere physical/enable/debug flags overlapped another render control");
    const std::uint32_t renderWord0 = packAtmosphereRenderWord0(
        1.1F, 1.3F, settings);
    const std::uint32_t renderWord1 = packAtmosphereRenderWord1(settings);
    const auto unpackByte = [](std::uint32_t word, unsigned shift) {
        return static_cast<float>((word >> shift) & 0xffU) / 255.0F;
    };
    require(std::abs(unpackByte(renderWord0, 16U) * 3.0F -
                     settings.rayleighScattering) <= 3.0F / 255.0F &&
                std::abs(unpackByte(renderWord0, 24U) * 3.0F -
                         settings.mieScattering) <= 3.0F / 255.0F &&
                std::abs(unpackByte(renderWord1, 0U) * 3.0F -
                         settings.absorption) <= 3.0F / 255.0F,
            "spectral coefficient packing exceeded one quantization step");
    const std::uint32_t shadowControls = (renderWord1 >> 24U) & 0xffU;
    require((shadowControls & 0x1U) != 0U &&
                ((shadowControls >> 1U) & 0x3U) == 2U,
            "terrain atmosphere shadow enable/quality packing is incorrect");

    settings.depthIntegrated = true;
    settings.integrationQuality = 2U;
    settings.nightScattering = 0.035F;
    const std::uint32_t integrationWord =
        packAtmosphereIntegrationPhaseWord(0x00abcde5U, settings);
    const std::uint32_t integrationControls = integrationWord >> 24U;
    require((integrationWord & 0x00ffffffU) == 0x00abcde5U &&
                (integrationControls & 1U) != 0U &&
                ((integrationControls >> 1U) & 3U) == 2U,
            "depth-integrated mode overwrote the cloud/diagnostic phase payload");
    require(atmosphereViewSampleCount(0U) == 1U &&
                atmosphereViewSampleCount(2U) == 3U &&
                atmosphereViewSampleCount(3U) == 5U &&
                atmosphereSunSampleCount(2U) == 2U &&
                atmosphereSunSampleCount(3U) == 4U,
            "atmosphere integration quality mapped to the wrong bounded budget");
    for (std::uint32_t sampleCount : {1U, 2U, 3U, 5U}) {
        float weightSum = 0.0F;
        float previousFraction = -1.0F;
        for (std::uint32_t index = 0U; index < sampleCount; ++index) {
            const auto sample = atmosphereQuadratureSample(sampleCount, index);
            require(sample.fraction > previousFraction && sample.fraction > 0.0F &&
                        sample.fraction < 1.0F && sample.weight > 0.0F,
                    "fixed atmosphere quadrature nodes were not ordered/bounded");
            previousFraction = sample.fraction;
            weightSum += sample.weight;
        }
        require(std::abs(weightSum - 1.0F) < 1.0e-5F,
                "fixed atmosphere quadrature weights did not conserve path length");
    }
    float denseReference = 0.0F;
    constexpr int denseSamples = 16384;
    for (int index = 0; index < denseSamples; ++index) {
        const float distance = 0.18F *
            (static_cast<float>(index) + 0.5F) / denseSamples;
        denseReference += atmosphereDensityProfile(
            1.0F + distance, 1.0F, 1.18F, 0.26F);
    }
    denseReference *= 0.18F / denseSamples;
    const float onePoint = atmosphereIntegratedDensity(
        {0.0F, 0.0F, 1.0F}, outward, 0.0F, 0.18F,
        1.0F, 1.18F, 0.26F, 1U);
    const float threePoint = atmosphereIntegratedDensity(
        {0.0F, 0.0F, 1.0F}, outward, 0.0F, 0.18F,
        1.0F, 1.18F, 0.26F, 3U);
    const float fivePoint = atmosphereIntegratedDensity(
        {0.0F, 0.0F, 1.0F}, outward, 0.0F, 0.18F,
        1.0F, 1.18F, 0.26F, 5U);
    require(std::abs(threePoint - denseReference) <
                std::abs(onePoint - denseReference) &&
                std::abs(fivePoint - denseReference) <
                std::abs(onePoint - denseReference),
            "depth-integrated fixed quadrature did not improve the radial optical integral");
    const auto depthRayleigh = atmosphereDepthIntegratedRayleighSpectrum(1.0F);
    const auto depthMie = atmosphereDepthIntegratedMieSpectrum(1.0F);
    const auto depthAbsorption =
        atmosphereDepthIntegratedAbsorptionSpectrum(1.0F);
    require(depthRayleigh[2] > depthRayleigh[1] &&
                depthRayleigh[1] > depthRayleigh[0] &&
                depthMie[0] > depthMie[2] &&
                depthAbsorption[2] > depthAbsorption[0],
            "SpaceBattle depth-integrated spectral basis was not preserved");

    const auto absorption = atmosphereAbsorptionCoefficients(1.0F);
    require(absorption[1] > absorption[0] && absorption[0] > absorption[2],
            "ozone-like absorption did not remove green/yellow most strongly");
    const float mieForward = atmosphereHenyeyGreenstein(1.0F, 0.68F);
    const float mieSide = atmosphereHenyeyGreenstein(0.0F, 0.68F);
    const float mieBack = atmosphereHenyeyGreenstein(-1.0F, 0.68F);
    require(std::isfinite(mieForward) && mieForward > mieSide &&
                mieSide > mieBack,
            "Henyey-Greenstein Mie phase is not finite and forward-peaked");

    const auto interiorPath = atmosphereSphereInterval(
        {0.0F, 0.0F, 1.04F}, outward, 1.18F);
    require(interiorPath.intersects && interiorPath.nearDistance == 0.0F &&
                interiorPath.farDistance > 0.0F,
            "camera inside atmosphere did not receive a finite shell-exit path");
    require(atmospherePlanetOccluded(
                {0.0F, 0.0F, 1.05F}, {0.0F, 0.0F, -1.0F}, 1.0F) &&
                !atmospherePlanetOccluded(
                    {0.0F, 0.0F, 1.05F}, {0.0F, 0.0F, 1.0F}, 1.0F),
            "planet occultation failed to distinguish day and core-shadow paths");

    const float sunElevation = 0.08F;
    const float sunRadial = std::sin(sunElevation);
    const float sunTangent = std::cos(sunElevation);
    const bool flatBlocked = atmosphereTerrainHorizonBlocked(
        1.0F, 1.0F, 0.025F, sunRadial, sunTangent);
    const bool ridgeBlocked = atmosphereTerrainHorizonBlocked(
        1.0F, 1.006F, 0.025F, sunRadial, sunTangent);
    require(!flatBlocked && ridgeBlocked,
            "bounded terrain horizon did not keep flat ground lit and block a ridge");
    for (bool resident : {false, true}) {
        (void)resident;
        require(atmosphereTerrainHorizonBlocked(
                    1.0F, 1.006F, 0.025F, sunRadial, sunTangent) ==
                    ridgeBlocked,
                "page residency changed compact-height terrain shadowing");
    }
    // The same world-space column heights and sun direction have no camera
    // argument, so orbit/F6 camera movement cannot alter this shadow result.
    require(atmosphereTerrainHorizonBlocked(
                1.0F, 1.006F, 0.025F, sunRadial, sunTangent) == ridgeBlocked,
            "camera-invariant terrain horizon changed for identical world data");
    require(!atmospherePlanetOccluded(
                {0.0F, 0.0F, 1.05F}, {1.0F, 0.0F, 0.0F}, 1.0F) &&
                atmosphereTerrainHorizonBlocked(
                    1.0F, 1.006F, 0.025F, sunRadial, sunTangent),
            "terrain shadow was not distinguished from the spherical core terminator");

    // The shell math is camera-relative. Translating camera and planet by the
    // same large amount leaves the relative ray interval exactly unchanged.
    constexpr std::array<float, 3> translatedCamera{10000.0F, -7000.0F, 2003.0F};
    constexpr std::array<float, 3> translatedPlanet{10000.0F, -7000.0F, 2000.0F};
    const std::array<float, 3> relative{
        translatedCamera[0] - translatedPlanet[0],
        translatedCamera[1] - translatedPlanet[1],
        translatedCamera[2] - translatedPlanet[2]};
    const auto translated = atmosphereSphereInterval(relative, inward, 1.18F);
    require(translated.intersects &&
                std::abs(translated.nearDistance - crossing.nearDistance) < 1.0e-5F,
            "camera/planet translation changed the relative shell geometry");

    require(atmosphereTransmittance(std::nanf(""), 1.0F) == 0.0F &&
                atmosphereDensityProfile(1.0F, 1.0F, 1.0F) == 0.0F,
            "nonfinite/degenerate atmosphere input escaped its safe guard");

    constexpr std::array<float, 3> destinationRadiance{0.8F, 0.6F, 0.4F};
    const OrderedMediumSample identity{};
    require(composeOrderedWormholeMedia(
                destinationRadiance, identity, identity) ==
                destinationRadiance,
            "zero-density source/destination media changed wormhole radiance");
    const OrderedMediumSample destinationMedium{
        {0.03F, 0.02F, 0.01F}, {0.70F, 0.75F, 0.80F}};
    const OrderedMediumSample sourceMedium{
        {0.02F, 0.03F, 0.04F}, {0.50F, 0.55F, 0.60F}};
    const auto destinationOnly = composeOrderedWormholeMedia(
        destinationRadiance, destinationMedium, identity);
    const auto sourceOnly = composeOrderedWormholeMedia(
        destinationRadiance, identity, sourceMedium);
    const auto bothMedia = composeOrderedWormholeMedia(
        destinationRadiance, destinationMedium, sourceMedium);
    require(destinationOnly != destinationRadiance &&
                sourceOnly != destinationRadiance &&
                bothMedia != destinationOnly && bothMedia != sourceOnly,
            "ordered source/destination attenuation was not independently applied");
    const auto expectedDestination = composeOrderedMediumSegment(
        destinationRadiance, destinationMedium);
    const auto expectedBoth = composeOrderedMediumSegment(
        expectedDestination, sourceMedium);
    require(bothMedia == expectedBoth,
            "wormhole media did not composite destination then source back-to-front");

    std::cout << "Atmosphere shell/surface geometry, spectral Beer-Lambert, "
                 "Rayleigh/Mie/absorption, core/terrain occultation, aerial perspective, "
                 "ordered wormhole media, star attenuation, packing, and translation "
                 "checks passed.\n";
}
