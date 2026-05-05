// Random - 随机数生成，PCG算法，质量高速度快

#pragma once

#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Core/MathConstants.h>
#include <cstdint>

namespace AstraAlgebra {

// PCG随机数生成器，又快又好
class PCGRandom {
public:
    PCGRandom(uint64_t seed = 0x853c49e6748fea9bULL, uint64_t inc = 0xda3e39cb94b95bdbULL);
    
    void seed(uint64_t newSeed);  // 换种子
    
    // 随机整数
    uint32_t nextUInt32();
    uint64_t nextUInt64();
    
    // [0, 1)的浮点数
    float nextFloat();
    double nextDouble();
    
    // [min, max)范围
    int nextInt(int min, int max);
    unsigned int nextUInt(unsigned int min, unsigned int max);
    float nextFloat(float min, float max);
    double nextDouble(double min, double max);
    
    // 布尔值
    bool nextBool();
    bool nextBool(float probability); // 指定概率，0.3就是30%返回true
    
    // 随机向量
    Vector2 nextVector2(float min, float max);
    Vector3 nextVector3(float min, float max);
    Vector4 nextVector4(float min, float max);
    
    // 单位球面上随机点
    Vector3 randomOnUnitSphere();
    Vector2 randomOnUnitCircle();
    
    // 单位球体内随机点
    Vector3 randomInUnitSphere();
    Vector2 randomInUnitCircle();
    
    // 随机方向（单位向量）
    Vector3 randomDirection();
    
    // 随机旋转（四元数）
    Quaternion randomRotation();
    
    // 随机颜色
    Vector3 randomColorRGB();
    
    // 从数组里随机选一个
    template<typename T>
    const T& choose(const T* array, size_t size) {
        return array[nextInt(0, (int)size)];
    }
    
    // 洗牌（Fisher-Yates算法）
    template<typename T>
    void shuffle(T* array, size_t size) {
        for (size_t i = size - 1; i > 0; --i) {
            size_t j = nextUInt64() % (i + 1);
            T temp = array[i];
            array[i] = array[j];
            array[j] = temp;
        }
    }

private:
    uint64_t state;
    uint64_t inc;
    
    uint32_t pcg32();
};

// 全局随机实例，直接用
extern PCGRandom g_Random;

// 便捷函数，不用自己搞实例
inline uint32_t RandomUInt32() { return g_Random.nextUInt32(); }
inline uint64_t RandomUInt64() { return g_Random.nextUInt64(); }
inline float RandomFloat() { return g_Random.nextFloat(); }
inline double RandomDouble() { return g_Random.nextDouble(); }
inline int RandomInt(int min, int max) { return g_Random.nextInt(min, max); }
inline float RandomFloat(float min, float max) { return g_Random.nextFloat(min, max); }
inline double RandomDouble(double min, double max) { return g_Random.nextDouble(min, max); }
inline bool RandomBool() { return g_Random.nextBool(); }
inline bool RandomBool(float probability) { return g_Random.nextBool(probability); }
inline Vector2 RandomVector2(float min, float max) { return g_Random.nextVector2(min, max); }
inline Vector3 RandomVector3(float min, float max) { return g_Random.nextVector3(min, max); }
inline Vector4 RandomVector4(float min, float max) { return g_Random.nextVector4(min, max); }
inline Vector3 RandomOnUnitSphere() { return g_Random.randomOnUnitSphere(); }
inline Vector2 RandomOnUnitCircle() { return g_Random.randomOnUnitCircle(); }
inline Vector3 RandomInUnitSphere() { return g_Random.randomInUnitSphere(); }
inline Vector2 RandomInUnitCircle() { return g_Random.randomInUnitCircle(); }
inline Vector3 RandomDirection() { return g_Random.randomDirection(); }
inline Quaternion RandomRotation() { return g_Random.randomRotation(); }
inline Vector3 RandomColorRGB() { return g_Random.randomColorRGB(); }

// 设置种子
inline void SeedRandom(uint64_t seed) { g_Random.seed(seed); }

// Perlin噪声生成器
class PerlinNoise {
public:
    PerlinNoise(uint32_t seed = 0);
    
    // 1D/2D/3D噪声，返回[-1, 1]
    float noise1D(float x) const;
    float noise2D(float x, float y) const;
    float noise3D(float x, float y, float z) const;
    
    // 分形布朗运动（多层叠加）
    float fbm1D(float x, int octaves, float persistence = 0.5f, float lacunarity = 2.0f) const;
    float fbm2D(float x, float y, int octaves, float persistence = 0.5f, float lacunarity = 2.0f) const;
    float fbm3D(float x, float y, float z, int octaves, float persistence = 0.5f, float lacunarity = 2.0f) const;
    
    // 湍流噪声
    float turbulence2D(float x, float y, int octaves, float persistence = 0.5f, float lacunarity = 2.0f) const;

private:
    uint8_t permutation[512];
    
    float fade(float t) const;
    float lerp(float t, float a, float b) const;
    float grad(int hash, float x, float y, float z) const;
};

// 随机分布工具
class RandomDistribution {
public:
    static float uniform(float min, float max);  // 均匀分布
    static float gaussian(float mean, float stddev);  // 正态分布（高斯）
    static float exponential(float lambda);  // 指数分布
    static float triangular(float min, float max, float mode);  // 三角分布
    static float beta(float alpha, float beta);  // 贝塔分布
    static int poisson(float lambda);  // 泊松分布

private:
    static PCGRandom& getGenerator();
};

} // namespace AstraAlgebra
