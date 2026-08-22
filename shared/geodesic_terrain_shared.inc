uint terrainAvalanche(uint value) {
    value ^= value >> 16u;
    value *= 0x7feb352du;
    value ^= value >> 15u;
    value *= 0x846ca68bu;
    value ^= value >> 16u;
    return value;
}

float terrainLatticeValue(ivec2 coordinate, uint seed) {
    uint bits = uint(coordinate.x) * 0x9e3779b9u;
    bits ^= uint(coordinate.y) * 0x85ebca6bu;
    bits ^= seed * 0xc2b2ae35u;
    return float(terrainAvalanche(bits) & 0x00ffffffu) / 16777215.0;
}

float terrainValueNoise(vec2 point, uint seed) {
    ivec2 cell = ivec2(floor(point));
    vec2 fraction = fract(point);
    vec2 blend = fraction * fraction * (3.0 - 2.0 * fraction);
    float bottom = mix(terrainLatticeValue(cell, seed),
                       terrainLatticeValue(cell + ivec2(1, 0), seed), blend.x);
    float top = mix(terrainLatticeValue(cell + ivec2(0, 1), seed),
                    terrainLatticeValue(cell + ivec2(1, 1), seed), blend.x);
    return mix(bottom, top, blend.y);
}

float terrainOrthographicNoise(vec3 direction, float scale, uint seed) {
    vec3 weights = abs(direction);
    weights *= weights;
    weights *= weights;
    weights /= max(weights.x + weights.y + weights.z, 1e-7);
    float projectedX = terrainValueNoise(direction.yz * scale + vec2(19.1, 7.7),
                                         seed ^ 0xa511e9b3u);
    float projectedY = terrainValueNoise(direction.xz * scale + vec2(-5.4, 13.8),
                                         seed ^ 0x63d83595u);
    float projectedZ = terrainValueNoise(direction.xy * scale + vec2(11.3, -17.2),
                                         seed ^ 0xb5297a4du);
    return dot(vec3(projectedX, projectedY, projectedZ), weights);
}

float terrainFractalNoise(vec3 direction, float scale, uint seed, float roughness) {
    float result = 0.0;
    float amplitude = 1.0;
    float amplitudeSum = 0.0;
    for (uint octave = 0u; octave < 4u; ++octave) {
        result += terrainOrthographicNoise(
            direction, scale, seed + octave * 0x9e3779b9u) * amplitude;
        amplitudeSum += amplitude;
        scale *= 2.03;
        amplitude *= roughness;
    }
    return result / max(amplitudeSum, 1e-6);
}

float terrainRidgedNoise(vec3 direction, float scale, uint seed, float roughness) {
    float result = 0.0;
    float amplitude = 1.0;
    float amplitudeSum = 0.0;
    for (uint octave = 0u; octave < 3u; ++octave) {
        float sampleValue = terrainOrthographicNoise(
            direction, scale, seed + octave * 0x68e31da4u);
        float ridge = 1.0 - abs(sampleValue * 2.0 - 1.0);
        result += ridge * ridge * amplitude;
        amplitudeSum += amplitude;
        scale *= 2.11;
        amplitude *= roughness;
    }
    return result / max(amplitudeSum, 1e-6);
}

void evaluateGeodesicTerrain(vec3 direction, out uint filledLayers,
                             out uint surfaceMaterial) {
    float roughness = clamp(frame.terrainRoughness, 0.15, 0.85);
    float continents = terrainFractalNoise(
        direction, max(frame.terrainContinentScale, 0.05),
        frame.terrainSeed, roughness);
    float mountains = terrainRidgedNoise(
        direction, max(frame.terrainMountainScale, 0.1),
        frame.terrainSeed ^ 0xd1b54a35u, roughness);
    float seaHeight = 0.36;
    float landSignal = continents - clamp(frame.terrainOceanLevel, 0.12, 0.88);
    float landMask = smoothstep(-0.065, 0.075, landSignal);
    float inlandMask = smoothstep(0.04, 0.24, landSignal);
    float mountainPeak = smoothstep(0.34, 0.86, mountains);
    float basin = (1.0 - mountains) * (1.0 - mountains) * inlandMask;
    float groundHeight = 0.12 + continents * 0.40;
    groundHeight += landSignal * frame.terrainContinentStrength * 0.55;
    groundHeight += mountainPeak * mountainPeak * landMask *
                    frame.terrainMountainStrength;
    groundHeight -= basin * frame.terrainContinentStrength * 0.22;
    groundHeight += pow(abs(direction.y), 7.0) * landMask * frame.terrainPolarStrength;
    float surfaceHeight = clamp(max(groundHeight, seaHeight), 0.08, 0.98);
    filledLayers = clamp(uint(round(surfaceHeight * 32.0)), 1u, 32u);
    surfaceMaterial = 1u;
    if (groundHeight < seaHeight) {
        surfaceMaterial = 3u;
    } else if (groundHeight < seaHeight + 0.055) {
        surfaceMaterial = 2u;
    } else if ((abs(direction.y) > 0.76 && groundHeight > seaHeight + 0.05) ||
               groundHeight > 0.80) {
        surfaceMaterial = 4u;
    } else if (mountainPeak > 0.64 && groundHeight > 0.58) {
        surfaceMaterial = 5u;
    }
}
