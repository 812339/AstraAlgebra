// SIMD - SIMD优化，批量运算的时候快很多，SSE/AVX那些

#pragma once

#include <AstraAlgebra/Core/MathUtils.h>

// 根据编译环境检测SIMD支持，AVX2 > SSE3 > SSE2，没有就回退到普通循环
#if defined(__AVX2__) || defined(__AVX__)
#include <immintrin.h>
#define ASTRAALGEBRA_SIMD_AVAILABLE
#define ASTRAALGEBRA_AVX2_AVAILABLE
#elif defined(__SSE3__) || defined(_M_X64) || defined(_M_IX86_FP)
#include <pmmintrin.h>
#include <emmintrin.h>
#define ASTRAALGEBRA_SIMD_AVAILABLE
#define ASTRAALGEBRA_SSE3_AVAILABLE
#elif defined(__SSE2__)
#include <emmintrin.h>
#define ASTRAALGEBRA_SIMD_AVAILABLE
#define ASTRAALGEBRA_SSE2_AVAILABLE
#elif defined(_MSC_VER)
#include <intrin.h>
#define ASTRAALGEBRA_SIMD_AVAILABLE
#if defined(__AVX2__) || defined(__AVX__)
#define ASTRAALGEBRA_AVX2_AVAILABLE
#elif defined(__SSE3__)
#define ASTRAALGEBRA_SSE3_AVAILABLE
#else
#define ASTRAALGEBRA_SSE2_AVAILABLE
#endif
#endif

namespace AstraAlgebra {

// SIMD命名空间，批量运算加速用的
namespace SIMD {

// SIMD向量类型定义，AVX2一次处理8个float，SSE一次4个
#ifdef ASTRAALGEBRA_AVX2_AVAILABLE
using Vec8f = __m256;
#endif

#ifdef ASTRAALGEBRA_SSE2_AVAILABLE
using Vec4f = __m128;
#endif

// 批量加法，SIMD一次算多个，快
inline void vectorAddBatch(const float* a, const float* b, float* result, int count) {
#ifdef ASTRAALGEBRA_AVX2_AVAILABLE
    int i = 0;
    for (; i <= count - 8; i += 8) {
        __m256 va = _mm256_loadu_ps(a + i);
        __m256 vb = _mm256_loadu_ps(b + i);
        __m256 vr = _mm256_add_ps(va, vb);
        _mm256_storeu_ps(result + i, vr);
    }
    for (; i < count; ++i) {
        result[i] = a[i] + b[i];
    }
#elif defined(ASTRAALGEBRA_SSE2_AVAILABLE)
    int i = 0;
    for (; i <= count - 4; i += 4) {
        __m128 va = _mm_loadu_ps(a + i);
        __m128 vb = _mm_loadu_ps(b + i);
        __m128 vr = _mm_add_ps(va, vb);
        _mm_storeu_ps(result + i, vr);
    }
    for (; i < count; ++i) {
        result[i] = a[i] + b[i];
    }
#else
    // 没有SIMD就普通循环
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] + b[i];
    }
#endif
}

// 批量减法
inline void vectorSubBatch(const float* a, const float* b, float* result, int count) {
#ifdef ASTRAALGEBRA_AVX2_AVAILABLE
    int i = 0;
    for (; i <= count - 8; i += 8) {
        __m256 va = _mm256_loadu_ps(a + i);
        __m256 vb = _mm256_loadu_ps(b + i);
        __m256 vr = _mm256_sub_ps(va, vb);
        _mm256_storeu_ps(result + i, vr);
    }
    for (; i < count; ++i) {
        result[i] = a[i] - b[i];
    }
#elif defined(ASTRAALGEBRA_SSE2_AVAILABLE)
    int i = 0;
    for (; i <= count - 4; i += 4) {
        __m128 va = _mm_loadu_ps(a + i);
        __m128 vb = _mm_loadu_ps(b + i);
        __m128 vr = _mm_sub_ps(va, vb);
        _mm_storeu_ps(result + i, vr);
    }
    for (; i < count; ++i) {
        result[i] = a[i] - b[i];
    }
#else
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] - b[i];
    }
#endif
}

// 批量乘法，分量对应相乘
inline void vectorMulBatch(const float* a, const float* b, float* result, int count) {
#ifdef ASTRAALGEBRA_AVX2_AVAILABLE
    int i = 0;
    for (; i <= count - 8; i += 8) {
        __m256 va = _mm256_loadu_ps(a + i);
        __m256 vb = _mm256_loadu_ps(b + i);
        __m256 vr = _mm256_mul_ps(va, vb);
        _mm256_storeu_ps(result + i, vr);
    }
    for (; i < count; ++i) {
        result[i] = a[i] * b[i];
    }
#elif defined(ASTRAALGEBRA_SSE2_AVAILABLE)
    int i = 0;
    for (; i <= count - 4; i += 4) {
        __m128 va = _mm_loadu_ps(a + i);
        __m128 vb = _mm_loadu_ps(b + i);
        __m128 vr = _mm_mul_ps(va, vb);
        _mm_storeu_ps(result + i, vr);
    }
    for (; i < count; ++i) {
        result[i] = a[i] * b[i];
    }
#else
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] * b[i];
    }
#endif
}

// 批量乘标量
inline void scalarMulBatch(const float* a, float scalar, float* result, int count) {
#ifdef ASTRAALGEBRA_AVX2_AVAILABLE
    __m256 vs = _mm256_set1_ps(scalar);
    int i = 0;
    for (; i <= count - 8; i += 8) {
        __m256 va = _mm256_loadu_ps(a + i);
        __m256 vr = _mm256_mul_ps(va, vs);
        _mm256_storeu_ps(result + i, vr);
    }
    for (; i < count; ++i) {
        result[i] = a[i] * scalar;
    }
#elif defined(ASTRAALGEBRA_SSE2_AVAILABLE)
    __m128 vs = _mm_set1_ps(scalar);
    int i = 0;
    for (; i <= count - 4; i += 4) {
        __m128 va = _mm_loadu_ps(a + i);
        __m128 vr = _mm_mul_ps(va, vs);
        _mm_storeu_ps(result + i, vr);
    }
    for (; i < count; ++i) {
        result[i] = a[i] * scalar;
    }
#else
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] * scalar;
    }
#endif
}

// 批量点积
inline void dotProductBatch(const float* a, const float* b, float* result, int count) {
#ifdef ASTRAALGEBRA_AVX2_AVAILABLE
    int i = 0;
    for (; i <= count - 8; i += 8) {
        __m256 va = _mm256_loadu_ps(a + i);
        __m256 vb = _mm256_loadu_ps(b + i);
        __m256 vmul = _mm256_mul_ps(va, vb);
        
        // 水平加法，把8个分量加起来
        __m128 low = _mm256_castps256_ps128(vmul);
        __m128 high = _mm256_extractf128_ps(vmul, 1);
        __m128 sum = _mm_add_ps(low, high);
        sum = _mm_hadd_ps(sum, sum);
        sum = _mm_hadd_ps(sum, sum);
        
        result[i] = _mm_cvtss_f32(sum);
    }
    for (; i < count; ++i) {
        result[i] = a[i] * b[i];
    }
#elif defined(ASTRAALGEBRA_SSE3_AVAILABLE) || defined(ASTRAALGEBRA_SSE2_AVAILABLE)
    int i = 0;
    for (; i <= count - 4; i += 4) {
        __m128 va = _mm_loadu_ps(a + i);
        __m128 vb = _mm_loadu_ps(b + i);
        __m128 vmul = _mm_mul_ps(va, vb);
        
        // 手动水平加法，SSE2兼容写法
        __m128 shuf = _mm_shuffle_ps(vmul, vmul, _MM_SHUFFLE(2, 3, 0, 1));
        __m128 sums = _mm_add_ps(vmul, shuf);
        shuf = _mm_movehl_ps(shuf, sums);
        sums = _mm_add_ss(sums, shuf);
        
        result[i] = _mm_cvtss_f32(sums);
    }
    for (; i < count; ++i) {
        result[i] = a[i] * b[i];
    }
#else
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] * b[i];
    }
#endif
}

// 批量归一化
inline void normalizeBatch(float* vectors, int vectorCount) {
#ifdef ASTRAALGEBRA_AVX2_AVAILABLE
    int i = 0;
    for (; i <= vectorCount - 2; i += 2) {
        float* v1 = vectors + i * 3;
        float* v2 = vectors + (i + 1) * 3;
        
        float len1 = Math::sqrt(v1[0] * v1[0] + v1[1] * v1[1] + v1[2] * v1[2]);
        float len2 = Math::sqrt(v2[0] * v2[0] + v2[1] * v2[1] + v2[2] * v2[2]);
        
        if (len1 > Math::EPSILON) {
            float inv1 = 1.0f / len1;
            v1[0] *= inv1;
            v1[1] *= inv1;
            v1[2] *= inv1;
        }
        if (len2 > Math::EPSILON) {
            float inv2 = 1.0f / len2;
            v2[0] *= inv2;
            v2[1] *= inv2;
            v2[2] *= inv2;
        }
    }
    for (; i < vectorCount; ++i) {
        float* v = vectors + i * 3;
        float len = Math::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (len > Math::EPSILON) {
            float inv = 1.0f / len;
            v[0] *= inv;
            v[1] *= inv;
            v[2] *= inv;
        }
    }
#else
    for (int i = 0; i < vectorCount; ++i) {
        float* v = vectors + i * 3;
        float len = Math::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (len > Math::EPSILON) {
            float inv = 1.0f / len;
            v[0] *= inv;
            v[1] *= inv;
            v[2] *= inv;
        }
    }
#endif
}

// 批量矩阵乘法，4x4矩阵，SIMD优化
inline void matrixMulBatch(const float* matricesA, const float* matricesB, float* result, int count) {
    for (int i = 0; i < count; ++i) {
        const float* a = matricesA + i * 16;
        const float* b = matricesB + i * 16;
        float* r = result + i * 16;
        
#ifdef ASTRAALGEBRA_SSE2_AVAILABLE
        // SIMD优化的矩阵乘法
        for (int row = 0; row < 4; ++row) {
            __m128 aRow = _mm_set1_ps(a[row * 4]);
            __m128 bCol0 = _mm_loadu_ps(b);
            __m128 resultRow = _mm_mul_ps(aRow, bCol0);
            
            aRow = _mm_set1_ps(a[row * 4 + 1]);
            bCol0 = _mm_loadu_ps(b + 4);
            resultRow = _mm_add_ps(resultRow, _mm_mul_ps(aRow, bCol0));
            
            aRow = _mm_set1_ps(a[row * 4 + 2]);
            bCol0 = _mm_loadu_ps(b + 8);
            resultRow = _mm_add_ps(resultRow, _mm_mul_ps(aRow, bCol0));
            
            aRow = _mm_set1_ps(a[row * 4 + 3]);
            bCol0 = _mm_loadu_ps(b + 12);
            resultRow = _mm_add_ps(resultRow, _mm_mul_ps(aRow, bCol0));
            
            _mm_storeu_ps(r + row * 4, resultRow);
        }
#else
        // 普通矩阵乘法
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += a[row * 4 + k] * b[k * 4 + col];
                }
                r[row * 4 + col] = sum;
            }
        }
#endif
    }
}

} // namespace SIMD

} // namespace AstraAlgebra
