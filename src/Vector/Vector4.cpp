// Vector4.cpp - 4D向量类实现
// 齐次坐标、颜色那些经常用

#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

// 添加SIMD支持的头文件
#ifdef __AVX2__
#include <immintrin.h>
#elif defined(__AVX__)
#include <immintrin.h>
#elif defined(__SSE2__)
#include <emmintrin.h>
#elif defined(_MSC_VER)
#include <immintrin.h>
#endif

namespace AstraAlgebra {

// 初始化静态常量
const Vector4 Vector4::zero(0.0f, 0.0f, 0.0f, 0.0f);
const Vector4 Vector4::one(1.0f, 1.0f, 1.0f, 1.0f);
const Vector4 Vector4::right(1.0f, 0.0f, 0.0f, 0.0f);
const Vector4 Vector4::left(-1.0f, 0.0f, 0.0f, 0.0f);
const Vector4 Vector4::up(0.0f, 1.0f, 0.0f, 0.0f);
const Vector4 Vector4::down(0.0f, -1.0f, 0.0f, 0.0f);
const Vector4 Vector4::forward(0.0f, 0.0f, 1.0f, 0.0f);
const Vector4 Vector4::back(0.0f, 0.0f, -1.0f, 0.0f);
const Vector4 Vector4::oneW(0.0f, 0.0f, 0.0f, 1.0f);

// 赋值操作符
Vector4& Vector4::operator=(const Vector4& other) {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
    }
    return *this;
}

Vector4& Vector4::operator=(Vector4&& other) noexcept {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
        other.x = 0.0f;
        other.y = 0.0f;
        other.z = 0.0f;
        other.w = 0.0f;
    }
    return *this;
}

Vector4& Vector4::operator=(const Vector2& other) {
    x = other.x;
    y = other.y;
    z = 0.0f;
    w = 1.0f;
    return *this;
}

Vector4& Vector4::operator=(const Vector3& other) {
    x = other.x;
    y = other.y;
    z = other.z;
    w = 1.0f;
    return *this;
}

Vector4& Vector4::operator+=(const Vector4& other) {
#if defined(__AVX2__)
    __m256 a = _mm256_load_ps(data);
    __m256 b = _mm256_load_ps(other.data);
    __m256 result = _mm256_add_ps(a, b);
    _mm256_store_ps(data, result);
#elif defined(__AVX__)
    __m256 a = _mm256_load_ps(data);
    __m256 b = _mm256_load_ps(other.data);
    __m256 result = _mm256_add_ps(a, b);
    _mm256_store_ps(data, result);
#elif defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_load_ps(data);
    __m128 b = _mm_load_ps(other.data);
    __m128 result = _mm_add_ps(a, b);
    _mm_store_ps(data, result);
#else
    x += other.x;
    y += other.y;
    z += other.z;
    w += other.w;
#endif
    return *this;
}

Vector4& Vector4::operator-=(const Vector4& other) {
#if defined(__AVX2__)
    __m256 a = _mm256_load_ps(data);
    __m256 b = _mm256_load_ps(other.data);
    __m256 result = _mm256_sub_ps(a, b);
    _mm256_store_ps(data, result);
#elif defined(__AVX__)
    __m256 a = _mm256_load_ps(data);
    __m256 b = _mm256_load_ps(other.data);
    __m256 result = _mm256_sub_ps(a, b);
    _mm256_store_ps(data, result);
#elif defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_load_ps(data);
    __m128 b = _mm_load_ps(other.data);
    __m128 result = _mm_sub_ps(a, b);
    _mm_store_ps(data, result);
#else
    x -= other.x;
    y -= other.y;
    z -= other.z;
    w -= other.w;
#endif
    return *this;
}

Vector4& Vector4::operator*=(float scalar) {
#if defined(__AVX2__)
    __m256 a = _mm256_load_ps(data);
    __m256 s = _mm256_set1_ps(scalar);
    __m256 result = _mm256_mul_ps(a, s);
    _mm256_store_ps(data, result);
#elif defined(__AVX__)
    __m256 a = _mm256_load_ps(data);
    __m256 s = _mm256_set1_ps(scalar);
    __m256 result = _mm256_mul_ps(a, s);
    _mm256_store_ps(data, result);
#elif defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_load_ps(data);
    __m128 s = _mm_set1_ps(scalar);
    __m128 result = _mm_mul_ps(a, s);
    _mm_store_ps(data, result);
#else
    x *= scalar;
    y *= scalar;
    z *= scalar;
    w *= scalar;
#endif
    return *this;
}

Vector4& Vector4::operator/=(float scalar) {
#if defined(__AVX2__)
    __m256 a = _mm256_load_ps(data);
    __m256 s = _mm256_set1_ps(1.0f / scalar);
    __m256 result = _mm256_mul_ps(a, s);
    _mm256_store_ps(data, result);
#elif defined(__AVX__)
    __m256 a = _mm256_load_ps(data);
    __m256 s = _mm256_set1_ps(1.0f / scalar);
    __m256 result = _mm256_mul_ps(a, s);
    _mm256_store_ps(data, result);
#elif defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_load_ps(data);
    __m128 s = _mm_set1_ps(1.0f / scalar);
    __m128 result = _mm_mul_ps(a, s);
    _mm_store_ps(data, result);
#else
    float invScalar = 1.0f / scalar;
    x *= invScalar;
    y *= invScalar;
    z *= invScalar;
    w *= invScalar;
#endif
    return *this;
}

Vector4& Vector4::normalize() {
    float len = length();
    if (len > Math::EPSILON) {
#if defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
        __m128 a = _mm_loadu_ps(&x);
        __m128 s = _mm_set1_ps(1.0f / len);
        __m128 result = _mm_mul_ps(a, s);
        _mm_storeu_ps(&x, result);
#else
        float invLen = 1.0f / len;
        x *= invLen;
        y *= invLen;
        z *= invLen;
        w *= invLen;
#endif
    }
    return *this;
}

Vector4 Vector4::project(const Vector4& other) const {
    float otherLenSq = other.lengthSquared();
    if (otherLenSq < Math::EPSILON) {
        return Vector4::zero;
    }
    float scale = dot(other) / otherLenSq;
    return other * scale;
}

Vector4 Vector4::reflect(const Vector4& normal) const {
    float dotProduct = dot(normal);
    return *this - normal * (2.0f * dotProduct);
}

Vector4 Vector4::slerp(const Vector4& other, float t) const {
    float dotProduct = dot(other);
    float lenProduct = length() * other.length();
    
    if (lenProduct < Math::EPSILON) {
        return lerp(other, t);
    }
    
    float cosAngle = Math::clamp(dotProduct / lenProduct, -1.0f, 1.0f);
    float angle = std::acos(cosAngle);
    float sinAngle = std::sin(angle);
    
    if (std::abs(sinAngle) < Math::EPSILON) {
        return lerp(other, t);
    }
    
    float t1 = std::sin((1.0f - t) * angle) / sinAngle;
    float t2 = std::sin(t * angle) / sinAngle;
    
    return *this * t1 + other * t2;
}

Vector4 Vector4::normalize(const Vector4& v) {
    return v.normalized();
}

Vector4 Vector4::slerp(const Vector4& a, const Vector4& b, float t) {
    return a.slerp(b, t);
}

std::ostream& operator<<(std::ostream& os, const Vector4& v) {
    os << "Vector4(" << v.x << ", " << v.y << ", " << v.z << ", " << v.w << ")";
    return os;
}

} // namespace AstraAlgebra
