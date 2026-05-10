// Vector3.cpp - 3D向量类实现
// 嗯，就是那个最常用的3D向量

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
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
const Vector3 Vector3::zero(0.0f, 0.0f, 0.0f);
const Vector3 Vector3::one(1.0f, 1.0f, 1.0f);
const Vector3 Vector3::right(1.0f, 0.0f, 0.0f);
const Vector3 Vector3::left(-1.0f, 0.0f, 0.0f);
const Vector3 Vector3::up(0.0f, 1.0f, 0.0f);
const Vector3 Vector3::down(0.0f, -1.0f, 0.0f);
const Vector3 Vector3::forward(0.0f, 0.0f, 1.0f);
const Vector3 Vector3::back(0.0f, 0.0f, -1.0f);

// 赋值操作符
Vector3& Vector3::operator=(const Vector3& other) {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
    }
    return *this;
}

Vector3& Vector3::operator=(Vector3&& other) noexcept {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        other.x = 0.0f;
        other.y = 0.0f;
        other.z = 0.0f;
    }
    return *this;
}

// 复合赋值操作符
Vector3& Vector3::operator+=(const Vector3& other) {
#if defined(__AVX2__) || defined(__AVX__) || defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_loadu_ps(data);
    __m128 b = _mm_loadu_ps(other.data);
    __m128 result = _mm_add_ps(a, b);
    _mm_storeu_ps(data, result);
#else
    x += other.x;
    y += other.y;
    z += other.z;
#endif
    return *this;
}

Vector3& Vector3::operator-=(const Vector3& other) {
#if defined(__AVX2__) || defined(__AVX__) || defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_loadu_ps(data);
    __m128 b = _mm_loadu_ps(other.data);
    __m128 result = _mm_sub_ps(a, b);
    _mm_storeu_ps(data, result);
#else
    x -= other.x;
    y -= other.y;
    z -= other.z;
#endif
    return *this;
}

Vector3& Vector3::operator*=(float scalar) {
#if defined(__AVX2__) || defined(__AVX__) || defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_loadu_ps(data);
    __m128 s = _mm_set1_ps(scalar);
    __m128 result = _mm_mul_ps(a, s);
    _mm_storeu_ps(data, result);
#else
    x *= scalar;
    y *= scalar;
    z *= scalar;
#endif
    return *this;
}

Vector3& Vector3::operator*=(const Vector3& other) {
#if defined(__AVX2__) || defined(__AVX__) || defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_loadu_ps(data);
    __m128 b = _mm_loadu_ps(other.data);
    __m128 result = _mm_mul_ps(a, b);
    _mm_storeu_ps(data, result);
#else
    x *= other.x;
    y *= other.y;
    z *= other.z;
#endif
    return *this;
}

Vector3& Vector3::operator/=(float scalar) {
    float invScalar = 1.0f / scalar;
    return (*this) *= invScalar;
}

Vector3& Vector3::operator/=(const Vector3& other) {
#if defined(__AVX2__) || defined(__AVX__) || defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    __m128 a = _mm_loadu_ps(data);
    __m128 b = _mm_loadu_ps(other.data);
    __m128 invB = _mm_div_ps(_mm_set1_ps(1.0f), b);
    __m128 result = _mm_mul_ps(a, invB);
    _mm_storeu_ps(data, result);
#else
    x /= other.x;
    y /= other.y;
    z /= other.z;
#endif
    return *this;
}

// 近似相等比较
bool Vector3::isApproximatelyEqual(const Vector3& other, float epsilon) const {
    return Math::approximatelyEqual(x, other.x, epsilon) &&
           Math::approximatelyEqual(y, other.y, epsilon) &&
           Math::approximatelyEqual(z, other.z, epsilon);
}

// 向量归一化
Vector3& Vector3::normalize() {
    float lenSq = x * x + y * y + z * z;
    if (lenSq > Math::EPSILON * Math::EPSILON) {
        float invLen = Math::fastInvSqrt(lenSq);
        x *= invLen; y *= invLen; z *= invLen;
    }
    return *this;
}

Vector3 Vector3::random(float min, float max) {
    return Vector3(
        Math::randomRange(min, max),
        Math::randomRange(min, max),
        Math::randomRange(min, max)
    );
}

Vector3 Vector3::randomDirection() {
    float theta = Math::randomRange(0.0f, Math::PI * 2.0f);
    float phi = Math::randomRange(0.0f, Math::PI);
    float sinPhi = Math::sin(phi);
    return Vector3(
        sinPhi * Math::sin(theta),
        Math::cos(phi),
        sinPhi * Math::cos(theta)
    ).normalized();
}

Vector3 Vector3::transform(const class Matrix4x4& matrix) const {
    return matrix * *this;
}

std::string Vector3::toString() const {
    char buffer[64];
    std::snprintf(buffer, sizeof(buffer), "Vector3(%.6f, %.6f, %.6f)", x, y, z);
    return std::string(buffer);
}

// 调试输出
std::ostream& operator<<(std::ostream& os, const Vector3& v) {
    os << "Vector3(" << v.x << ", " << v.y << ", " << v.z << ")";
    return os;
}

} // namespace AstraAlgebra
