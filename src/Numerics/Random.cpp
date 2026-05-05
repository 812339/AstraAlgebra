// Random.cpp - 随机数生成工具实现
// PCG算法，质量高速度快

#include <AstraAlgebra/Numerics/Random.h>
#include <cmath>

namespace AstraAlgebra {

// PCG32实现
PCGRandom::PCGRandom(uint64_t seed, uint64_t initInc) : state(0), inc(0) {
    inc = (initInc << 1) | 1;
    nextUInt32();
    state += seed;
    nextUInt32();
}

void PCGRandom::seed(uint64_t newSeed) {
    state = 0;
    inc = 0xda3e39cb94b95bdbULL;
    nextUInt32();
    state += newSeed;
    nextUInt32();
}

uint32_t PCGRandom::pcg32() {
    uint64_t oldState = state;
    state = oldState * 6364136223846793005ULL + inc;
    uint32_t xorshifted = ((oldState >> 18u) ^ oldState) >> 27u;
    uint32_t rot = oldState >> 59u;
    return (xorshifted >> rot) | (xorshifted << ((-rot) & 31));
}

uint32_t PCGRandom::nextUInt32() {
    return pcg32();
}

uint64_t PCGRandom::nextUInt64() {
    return ((uint64_t)pcg32() << 32) | pcg32();
}

float PCGRandom::nextFloat() {
    return (pcg32() >> 8) * (1.0f / 16777216.0f);
}

double PCGRandom::nextDouble() {
    return (((uint64_t)pcg32() << 20) | (pcg32() >> 12)) * (1.0 / 9007199254740992.0);
}

int PCGRandom::nextInt(int min, int max) {
    unsigned int range = (unsigned int)(max - min);
    unsigned int threshold = (0xFFFFFFFFu - range) % range;
    unsigned int r;
    do {
        r = pcg32();
    } while (r <= threshold);
    return min + (int)(r % range);
}

unsigned int PCGRandom::nextUInt(unsigned int min, unsigned int max) {
    unsigned int range = max - min;
    unsigned int threshold = (0xFFFFFFFFu - range) % range;
    unsigned int r;
    do {
        r = pcg32();
    } while (r <= threshold);
    return min + (r % range);
}

float PCGRandom::nextFloat(float min, float max) {
    return min + nextFloat() * (max - min);
}

double PCGRandom::nextDouble(double min, double max) {
    return min + nextDouble() * (max - min);
}

bool PCGRandom::nextBool() {
    return (pcg32() & 0x80000000) != 0;
}

bool PCGRandom::nextBool(float probability) {
    return nextFloat() < probability;
}

Vector2 PCGRandom::nextVector2(float min, float max) {
    return Vector2(nextFloat(min, max), nextFloat(min, max));
}

Vector3 PCGRandom::nextVector3(float min, float max) {
    return Vector3(nextFloat(min, max), nextFloat(min, max), nextFloat(min, max));
}

Vector4 PCGRandom::nextVector4(float min, float max) {
    return Vector4(nextFloat(min, max), nextFloat(min, max), nextFloat(min, max), nextFloat(min, max));
}

Vector3 PCGRandom::randomOnUnitSphere() {
    float u = nextFloat() * 2.0f - 1.0f;
    float t = nextFloat() * Math::TWO_PI;
    float f = std::sqrt(1.0f - u * u);
    return Vector3(f * std::cos(t), f * std::sin(t), u);
}

Vector2 PCGRandom::randomOnUnitCircle() {
    float angle = nextFloat() * Math::TWO_PI;
    return Vector2(std::cos(angle), std::sin(angle));
}

Vector3 PCGRandom::randomInUnitSphere() {
    Vector3 v;
    do {
        v = nextVector3(-1.0f, 1.0f);
    } while (v.lengthSquared() >= 1.0f);
    return v;
}

Vector2 PCGRandom::randomInUnitCircle() {
    Vector2 v;
    do {
        v = nextVector2(-1.0f, 1.0f);
    } while (v.lengthSquared() >= 1.0f);
    return v;
}

Vector3 PCGRandom::randomDirection() {
    return randomOnUnitSphere().normalized();
}

Quaternion PCGRandom::randomRotation() {
    float u1 = nextFloat();
    float u2 = nextFloat() * Math::TWO_PI;
    float u3 = nextFloat() * Math::TWO_PI;
    float sq1 = std::sqrt(1.0f - u1);
    float sq2 = std::sqrt(u1);
    float theta1 = Math::PI_2 * u2;
    float theta2 = Math::PI_2 * u3;
    return Quaternion(sq1 * std::sin(theta1), sq1 * std::cos(theta1), sq2 * std::sin(theta2), sq2 * std::cos(theta2));
}

Vector3 PCGRandom::randomColorRGB() {
    return Vector3(nextFloat(), nextFloat(), nextFloat());
}

// 全局随机实例
PCGRandom g_Random;

// Perlin噪声实现
PerlinNoise::PerlinNoise(uint32_t seed) {
    PCGRandom rng(seed);
    for (int i = 0; i < 256; ++i) {
        permutation[i] = (uint8_t)i;
    }
    rng.shuffle(permutation, 256);
    for (int i = 0; i < 256; ++i) {
        permutation[256 + i] = permutation[i];
    }
}

float PerlinNoise::fade(float t) const {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

float PerlinNoise::lerp(float t, float a, float b) const {
    return a + t * (b - a);
}

float PerlinNoise::grad(int hash, float x, float y, float z) const {
    int h = hash & 15;
    float u = h < 8 ? x : y;
    float v = h < 4 ? y : (h == 12 || h == 14 ? x : z);
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

float PerlinNoise::noise1D(float x) const {
    int xi = (int)std::floor(x) & 255;
    float xf = x - std::floor(x);
    float u = fade(xf);
    
    int aa = permutation[xi];
    int ab = permutation[xi + 1];
    
    return lerp(u, grad(aa, xf, 0.0f, 0.0f), grad(ab, xf - 1.0f, 0.0f, 0.0f));
}

float PerlinNoise::noise2D(float x, float y) const {
    int xi = (int)std::floor(x) & 255;
    int yi = (int)std::floor(y) & 255;
    float xf = x - std::floor(x);
    float yf = y - std::floor(y);
    
    float u = fade(xf);
    float v = fade(yf);
    
    int aa = permutation[xi] + yi;
    int ab = permutation[xi + 1] + yi;
    
    return lerp(v,
        lerp(u, grad(permutation[aa], xf, yf, 0.0f), grad(permutation[ab], xf - 1.0f, yf, 0.0f)),
        lerp(u, grad(permutation[aa + 1], xf, yf - 1.0f, 0.0f), grad(permutation[ab + 1], xf - 1.0f, yf - 1.0f, 0.0f))
    );
}

float PerlinNoise::noise3D(float x, float y, float z) const {
    int xi = (int)std::floor(x) & 255;
    int yi = (int)std::floor(y) & 255;
    int zi = (int)std::floor(z) & 255;
    float xf = x - std::floor(x);
    float yf = y - std::floor(y);
    float zf = z - std::floor(z);
    
    float u = fade(xf);
    float v = fade(yf);
    float w = fade(zf);
    
    int aaa = permutation[permutation[permutation[xi] + yi] + zi];
    int aba = permutation[permutation[permutation[xi + 1] + yi] + zi];
    int aab = permutation[permutation[permutation[xi] + yi + 1] + zi];
    int abb = permutation[permutation[permutation[xi + 1] + yi + 1] + zi];
    int baa = permutation[permutation[permutation[xi] + yi] + zi + 1];
    int bba = permutation[permutation[permutation[xi + 1] + yi] + zi + 1];
    int bab = permutation[permutation[permutation[xi] + yi + 1] + zi + 1];
    int bbb = permutation[permutation[permutation[xi + 1] + yi + 1] + zi + 1];
    
    return lerp(w,
        lerp(v,
            lerp(u, grad(aaa, xf, yf, zf), grad(aba, xf - 1.0f, yf, zf)),
            lerp(u, grad(aab, xf, yf - 1.0f, zf), grad(abb, xf - 1.0f, yf - 1.0f, zf))),
        lerp(v,
            lerp(u, grad(baa, xf, yf, zf - 1.0f), grad(bba, xf - 1.0f, yf, zf - 1.0f)),
            lerp(u, grad(bab, xf, yf - 1.0f, zf - 1.0f), grad(bbb, xf - 1.0f, yf - 1.0f, zf - 1.0f)))
    );
}

float PerlinNoise::fbm1D(float x, int octaves, float persistence, float lacunarity) const {
    float total = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float maxValue = 0.0f;
    
    for (int i = 0; i < octaves; ++i) {
        total += noise1D(x * frequency) * amplitude;
        maxValue += amplitude;
        amplitude *= persistence;
        frequency *= lacunarity;
    }
    
    return total / maxValue;
}

float PerlinNoise::fbm2D(float x, float y, int octaves, float persistence, float lacunarity) const {
    float total = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float maxValue = 0.0f;
    
    for (int i = 0; i < octaves; ++i) {
        total += noise2D(x * frequency, y * frequency) * amplitude;
        maxValue += amplitude;
        amplitude *= persistence;
        frequency *= lacunarity;
    }
    
    return total / maxValue;
}

float PerlinNoise::fbm3D(float x, float y, float z, int octaves, float persistence, float lacunarity) const {
    float total = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float maxValue = 0.0f;
    
    for (int i = 0; i < octaves; ++i) {
        total += noise3D(x * frequency, y * frequency, z * frequency) * amplitude;
        maxValue += amplitude;
        amplitude *= persistence;
        frequency *= lacunarity;
    }
    
    return total / maxValue;
}

float PerlinNoise::turbulence2D(float x, float y, int octaves, float persistence, float lacunarity) const {
    float total = 0.0f;
    float amplitude = 1.0f;
    float frequency = 1.0f;
    float maxValue = 0.0f;
    
    for (int i = 0; i < octaves; ++i) {
        total += std::abs(noise2D(x * frequency, y * frequency)) * amplitude;
        maxValue += amplitude;
        amplitude *= persistence;
        frequency *= lacunarity;
    }
    
    return total / maxValue;
}

// 随机分布实现
float RandomDistribution::uniform(float min, float max) {
    return g_Random.nextFloat(min, max);
}

float RandomDistribution::gaussian(float mean, float stddev) {
    float u1 = g_Random.nextFloat();
    float u2 = g_Random.nextFloat();
    float z = std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * Math::PI * u2);
    return mean + stddev * z;
}

float RandomDistribution::exponential(float lambda) {
    float u = g_Random.nextFloat();
    return -std::log(1.0f - u) / lambda;
}

float RandomDistribution::triangular(float min, float max, float mode) {
    float u = g_Random.nextFloat();
    float fc = (mode - min) / (max - min);
    if (u < fc) {
        return min + std::sqrt(u * (max - min) * (mode - min));
    } else {
        return max - std::sqrt((1.0f - u) * (max - min) * (max - mode));
    }
}

float RandomDistribution::beta(float alpha, float beta) {
    // 简化实现：使用gamma分布近似
    float x = 0.0f;
    for (int i = 0; i < (int)alpha; ++i) {
        x += exponential(1.0f);
    }
    float y = 0.0f;
    for (int i = 0; i < (int)beta; ++i) {
        y += exponential(1.0f);
    }
    return x / (x + y);
}

int RandomDistribution::poisson(float lambda) {
    float L = std::exp(-lambda);
    int k = 0;
    float p = 1.0f;
    do {
        k++;
        p *= g_Random.nextFloat();
    } while (p > L);
    return k - 1;
}

PCGRandom& RandomDistribution::getGenerator() {
    return g_Random;
}

} // namespace AstraAlgebra
