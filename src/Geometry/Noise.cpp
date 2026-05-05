// Noise.cpp - 噪声生成类实现
// Perlin噪声、Simplex噪声那些，地形生成经常用

#include <AstraAlgebra/Geometry/Noise.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cstdlib>
#include <cmath>
#include <algorithm>

namespace AstraAlgebra {

// 构造函数
Noise::Noise() {
    initPermutationTable(0);  // 使用默认种子0
}

Noise::Noise(unsigned int seed) {
    initPermutationTable(seed);
}

// 初始化排列表
void Noise::initPermutationTable(unsigned int seed) {
    // 初始化基础排列表
    for (int i = 0; i < PERMUTATION_TABLE_SIZE; ++i) {
        permutationTable[i] = i;
    }
    
    // 使用种子打乱排列表
    std::srand(seed);
    for (int i = PERMUTATION_TABLE_SIZE - 1; i > 0; --i) {
        int j = std::rand() % (i + 1);
        std::swap(permutationTable[i], permutationTable[j]);
    }
    
    // 扩展排列表，便于循环访问
    for (int i = 0; i < PERMUTATION_TABLE_SIZE; ++i) {
        permutationTable[PERMUTATION_TABLE_SIZE + i] = permutationTable[i];
    }
}

// 平滑插值函数
auto Noise::fade(float t) const -> float {
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}

// 线性插值函数
auto Noise::lerp(float t, float a, float b) const -> float {
    return a + t * (b - a);
}

// Perlin噪声的梯度函数
auto Noise::grad(int hash, float x, float y, float z, float w) const -> float {
    // 根据hash值选择梯度向量
    int h = hash & 31;  // 0-31
    
    // 4D梯度向量，每个分量为-1或1
    float u = h < 24 ? (h < 12 ? x : y) : z;
    float v = h < 8 ? y : (h < 16 ? z : (h < 24 ? w : x));
    
    return ((h & 1) == 0 ? u : -u) + ((h & 2) == 0 ? v : -v);
}

// 2D Perlin噪声
auto Noise::perlin2D(const Vector2& point) const -> float {
    return perlin2D(point.x, point.y);
}

auto Noise::perlin2D(float x, float y) const -> float {
    // 计算整数坐标
    int X = static_cast<int>(std::floor(x)) & (PERMUTATION_TABLE_SIZE - 1);
    int Y = static_cast<int>(std::floor(y)) & (PERMUTATION_TABLE_SIZE - 1);
    
    // 计算小数部分
    x -= std::floor(x);
    y -= std::floor(y);
    
    // 计算平滑插值因子
    float u = fade(x);
    float v = fade(y);
    
    // 哈希坐标
    int A = permutationTable[X] + Y;
    int AA = permutationTable[A];
    int AB = permutationTable[A + 1];
    int B = permutationTable[X + 1] + Y;
    int BA = permutationTable[B];
    int BB = permutationTable[B + 1];
    
    // 计算梯度贡献并插值
    float gradAA = grad(permutationTable[AA], x, y, 0.0f);
    float gradBA = grad(permutationTable[BA], x - 1.0f, y, 0.0f);
    float gradAB = grad(permutationTable[AB], x, y - 1.0f, 0.0f);
    float gradBB = grad(permutationTable[BB], x - 1.0f, y - 1.0f, 0.0f);
    
    float lerpX1 = lerp(u, gradAA, gradBA);
    float lerpX2 = lerp(u, gradAB, gradBB);
    float result = lerp(v, lerpX1, lerpX2);
    
    // 将结果映射到[-1, 1]范围
    return result;
}

// 3D Perlin噪声
auto Noise::perlin3D(const Vector3& point) const -> float {
    return perlin3D(point.x, point.y, point.z);
}

auto Noise::perlin3D(float x, float y, float z) const -> float {
    // 计算整数坐标
    int X = static_cast<int>(std::floor(x)) & (PERMUTATION_TABLE_SIZE - 1);
    int Y = static_cast<int>(std::floor(y)) & (PERMUTATION_TABLE_SIZE - 1);
    int Z = static_cast<int>(std::floor(z)) & (PERMUTATION_TABLE_SIZE - 1);
    
    // 计算小数部分
    x -= std::floor(x);
    y -= std::floor(y);
    z -= std::floor(z);
    
    // 计算平滑插值因子
    float u = fade(x);
    float v = fade(y);
    float w = fade(z);
    
    // 哈希坐标
    int A = permutationTable[X] + Y;
    int AA = permutationTable[A] + Z;
    int AB = permutationTable[A + 1] + Z;
    int B = permutationTable[X + 1] + Y;
    int BA = permutationTable[B] + Z;
    int BB = permutationTable[B + 1] + Z;
    
    // 计算梯度贡献并插值
    float gradAA = grad(permutationTable[AA], x, y, z);
    float gradBA = grad(permutationTable[BA], x - 1.0f, y, z);
    float gradAB = grad(permutationTable[AB], x, y - 1.0f, z);
    float gradBB = grad(permutationTable[BB], x - 1.0f, y - 1.0f, z);
    
    float lerpX1 = lerp(u, gradAA, gradBA);
    float lerpX2 = lerp(u, gradAB, gradBB);
    float lerpY1 = lerp(v, lerpX1, lerpX2);
    
    float gradAA1 = grad(permutationTable[AA + 1], x, y, z - 1.0f);
    float gradBA1 = grad(permutationTable[BA + 1], x - 1.0f, y, z - 1.0f);
    float gradAB1 = grad(permutationTable[AB + 1], x, y - 1.0f, z - 1.0f);
    float gradBB1 = grad(permutationTable[BB + 1], x - 1.0f, y - 1.0f, z - 1.0f);
    
    float lerpX3 = lerp(u, gradAA1, gradBA1);
    float lerpX4 = lerp(u, gradAB1, gradBB1);
    float lerpY2 = lerp(v, lerpX3, lerpX4);
    
    float result = lerp(w, lerpY1, lerpY2);
    
    // 将结果映射到[-1, 1]范围
    return result;
}

// 4D Perlin噪声
auto Noise::perlin4D(const Vector4& point) const -> float {
    return perlin4D(point.x, point.y, point.z, point.w);
}

auto Noise::perlin4D(float x, float y, float z, float w) const -> float {
    // 计算整数坐标
    int X = static_cast<int>(std::floor(x)) & (PERMUTATION_TABLE_SIZE - 1);
    int Y = static_cast<int>(std::floor(y)) & (PERMUTATION_TABLE_SIZE - 1);
    int Z = static_cast<int>(std::floor(z)) & (PERMUTATION_TABLE_SIZE - 1);
    int W = static_cast<int>(std::floor(w)) & (PERMUTATION_TABLE_SIZE - 1);
    
    // 计算小数部分
    x -= std::floor(x);
    y -= std::floor(y);
    z -= std::floor(z);
    w -= std::floor(w);
    
    // 计算平滑插值因子
    float u = fade(x);
    float v = fade(y);
    float s = fade(z);
    float t = fade(w);
    
    // 哈希坐标
    int A = permutationTable[X] + Y;
    int AA = permutationTable[A] + Z;
    int AB = permutationTable[A + 1] + Z;
    int B = permutationTable[X + 1] + Y;
    int BA = permutationTable[B] + Z;
    int BB = permutationTable[B + 1] + Z;
    
    int AAA = permutationTable[AA] + W;
    int ABA = permutationTable[AB] + W;
    int BAA = permutationTable[BA] + W;
    int BBA = permutationTable[BB] + W;
    int AAB = permutationTable[AA + 1] + W;
    int ABB = permutationTable[AB + 1] + W;
    int BAB = permutationTable[BA + 1] + W;
    int BBB = permutationTable[BB + 1] + W;
    
    // 计算梯度贡献并插值
    float gradAAA = grad(permutationTable[AAA], x, y, z, w);
    float gradBAA = grad(permutationTable[BAA], x - 1.0f, y, z, w);
    float gradABA = grad(permutationTable[ABA], x, y - 1.0f, z, w);
    float gradBBA = grad(permutationTable[BBA], x - 1.0f, y - 1.0f, z, w);
    
    float gradAAB = grad(permutationTable[AAB], x, y, z - 1.0f, w);
    float gradBAB = grad(permutationTable[BAB], x - 1.0f, y, z - 1.0f, w);
    float gradABB = grad(permutationTable[ABB], x, y - 1.0f, z - 1.0f, w);
    float gradBBB = grad(permutationTable[BBB], x - 1.0f, y - 1.0f, z - 1.0f, w);
    
    // 插值计算
    float lerpX1 = lerp(u, gradAAA, gradBAA);
    float lerpX2 = lerp(u, gradABA, gradBBA);
    float lerpX3 = lerp(u, gradAAB, gradBAB);
    float lerpX4 = lerp(u, gradABB, gradBBB);
    
    float lerpY1 = lerp(v, lerpX1, lerpX2);
    float lerpY2 = lerp(v, lerpX3, lerpX4);
    
    float lerpZ1 = lerp(s, lerpY1, lerpY2);
    
    // 处理w方向的插值
    float gradAAA1 = grad(permutationTable[AAA + 1], x, y, z, w - 1.0f);
    float gradBAA1 = grad(permutationTable[BAA + 1], x - 1.0f, y, z, w - 1.0f);
    float gradABA1 = grad(permutationTable[ABA + 1], x, y - 1.0f, z, w - 1.0f);
    float gradBBA1 = grad(permutationTable[BBA + 1], x - 1.0f, y - 1.0f, z, w - 1.0f);
    
    float gradAAB1 = grad(permutationTable[AAB + 1], x, y, z - 1.0f, w - 1.0f);
    float gradBAB1 = grad(permutationTable[BAB + 1], x - 1.0f, y, z - 1.0f, w - 1.0f);
    float gradABB1 = grad(permutationTable[ABB + 1], x, y - 1.0f, z - 1.0f, w - 1.0f);
    float gradBBB1 = grad(permutationTable[BBB + 1], x - 1.0f, y - 1.0f, z - 1.0f, w - 1.0f);
    
    float lerpX5 = lerp(u, gradAAA1, gradBAA1);
    float lerpX6 = lerp(u, gradABA1, gradBBA1);
    float lerpX7 = lerp(u, gradAAB1, gradBAB1);
    float lerpX8 = lerp(u, gradABB1, gradBBB1);
    
    float lerpY3 = lerp(v, lerpX5, lerpX6);
    float lerpY4 = lerp(v, lerpX7, lerpX8);
    
    float lerpZ2 = lerp(s, lerpY3, lerpY4);
    
    float result = lerp(t, lerpZ1, lerpZ2);
    
    // 将结果映射到[-1, 1]范围
    return result;
}

// 分形Perlin噪声（多层叠加）
auto Noise::fractalPerlin2D(const Vector2& point, int octaves, float persistence) const -> float {
    return fractalPerlin2D(point.x, point.y, octaves, persistence);
}

auto Noise::fractalPerlin2D(float x, float y, int octaves, float persistence) const -> float {
    float total = 0.0f;
    float frequency = 1.0f;
    float amplitude = 1.0f;
    float maxValue = 0.0f;
    
    for (int i = 0; i < octaves; ++i) {
        total += perlin2D(x * frequency, y * frequency) * amplitude;
        maxValue += amplitude;
        amplitude *= persistence;
        frequency *= 2.0f;
    }
    
    // 将结果归一化到[-1, 1]范围
    return total / maxValue;
}

auto Noise::fractalPerlin3D(const Vector3& point, int octaves, float persistence) const -> float {
    return fractalPerlin3D(point.x, point.y, point.z, octaves, persistence);
}

auto Noise::fractalPerlin3D(float x, float y, float z, int octaves, float persistence) const -> float {
    float total = 0.0f;
    float frequency = 1.0f;
    float amplitude = 1.0f;
    float maxValue = 0.0f;
    
    for (int i = 0; i < octaves; ++i) {
        total += perlin3D(x * frequency, y * frequency, z * frequency) * amplitude;
        maxValue += amplitude;
        amplitude *= persistence;
        frequency *= 2.0f;
    }
    
    // 将结果归一化到[-1, 1]范围
    return total / maxValue;
}

// 2D Simplex噪声（简化实现，基于Perlin噪声）
auto Noise::simplex2D(const Vector2& point) const -> float {
    return simplex2D(point.x, point.y);
}

auto Noise::simplex2D(float x, float y) const -> float {
    // 简化实现：使用Perlin噪声代替Simplex噪声
    // 注意：这里可以实现完整的Simplex噪声算法
    return perlin2D(x, y);
}

// 3D Simplex噪声（简化实现，基于Perlin噪声）
auto Noise::simplex3D(const Vector3& point) const -> float {
    return simplex3D(point.x, point.y, point.z);
}

auto Noise::simplex3D(float x, float y, float z) const -> float {
    // 简化实现：使用Perlin噪声代替Simplex噪声
    // 注意：这里可以实现完整的Simplex噪声算法
    return perlin3D(x, y, z);
}

// 设置种子
void Noise::setSeed(unsigned int seed) {
    initPermutationTable(seed);
}

} // namespace AstraAlgebra