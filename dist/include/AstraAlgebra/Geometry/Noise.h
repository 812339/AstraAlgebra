// Noise - 噪声生成，Perlin噪声、Simplex噪声那些，地形生成经常用

#pragma once

#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>

namespace AstraAlgebra {

// 噪声类，生成各种噪声，做地形、纹理啥的很好用
class Noise {
public:
    Noise();  // 默认，随机种子
    Noise(unsigned int seed);  // 给个种子
    
    // 2D Perlin噪声
    float perlin2D(const Vector2& point) const;
    float perlin2D(float x, float y) const;
    
    // 3D Perlin噪声
    float perlin3D(const Vector3& point) const;
    float perlin3D(float x, float y, float z) const;
    
    // 4D Perlin噪声
    float perlin4D(const Vector4& point) const;
    float perlin4D(float x, float y, float z, float w) const;
    
    // 2D Simplex噪声（比Perlin快一点）
    float simplex2D(const Vector2& point) const;
    float simplex2D(float x, float y) const;
    
    // 3D Simplex噪声
    float simplex3D(const Vector3& point) const;
    float simplex3D(float x, float y, float z) const;
    
    // 分形噪声（多层叠加，更自然）
    float fractalPerlin2D(const Vector2& point, int octaves, float persistence) const;
    float fractalPerlin2D(float x, float y, int octaves, float persistence) const;
    
    float fractalPerlin3D(const Vector3& point, int octaves, float persistence) const;
    float fractalPerlin3D(float x, float y, float z, int octaves, float persistence) const;
    
    // 换个种子
    void setSeed(unsigned int seed);
    
private:
    // Perlin噪声的梯度表
    static const int PERMUTATION_TABLE_SIZE = 256;
    int permutationTable[PERMUTATION_TABLE_SIZE * 2];  // 排列表，搞大点方便循环
    
    // 初始化排列表
    void initPermutationTable(unsigned int seed);
    
    // 梯度函数
    float grad(int hash, float x, float y, float z, float w = 0.0f) const;
    
    // 平滑函数
    float fade(float t) const;
    
    // 线性插值
    float lerp(float t, float a, float b) const;
};

} // namespace AstraAlgebra
