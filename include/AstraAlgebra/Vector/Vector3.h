// Vector3 - 3D向量

#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <iostream>
#include <cmath>

namespace AstraAlgebra {

// 3D向量，嗯。用union是为了后面和SIMD对齐方便，让我想想...应该吧
class Vector3 {
public:
#ifdef __GNUC__
#pragma GCC diagnostic push
#pragma GCC diagnostic ignored "-Wpedantic"
#endif
    union {
        struct {
            float x, y, z, w;
        };
        float data[4];
    };
#ifdef __GNUC__
#pragma GCC diagnostic pop
#endif
    
    void set(float x, float y, float z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    // 构造函数
    constexpr Vector3() : x(0.0f), y(0.0f), z(0.0f), w(0.0f) {}
    constexpr Vector3(float x, float y, float z) : x(x), y(y), z(z), w(0.0f) {}
    constexpr Vector3(float scalar) : x(scalar), y(scalar), z(scalar), w(0.0f) {}
    constexpr Vector3(const Vector3& other) : x(other.x), y(other.y), z(other.z), w(0.0f) {}
    constexpr Vector3(Vector3&& other) noexcept : x(other.x), y(other.y), z(other.z), w(0.0f) {
        other.x = 0.0f;
        other.y = 0.0f;
        other.z = 0.0f;
        other.w = 0.0f;
    }

    ~Vector3() = default;

    Vector3& operator=(const Vector3& other);
    Vector3& operator=(Vector3&& other) noexcept;

    constexpr float& operator[](int index) { return (&x)[index]; }
    constexpr const float& operator[](int index) const { return (&x)[index]; }

    constexpr Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }
    constexpr Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }
    constexpr Vector3 operator*(float scalar) const {
        return Vector3(x * scalar, y * scalar, z * scalar);
    }
    constexpr Vector3 operator*(const Vector3& other) const {
        return Vector3(x * other.x, y * other.y, z * other.z);
    }
    constexpr Vector3 operator/(float scalar) const {
        float invScalar = 1.0f / scalar;
        return (*this) * invScalar;
    }
    constexpr Vector3 operator/(const Vector3& other) const {
        return Vector3(x / other.x, y / other.y, z / other.z);
    }

    Vector3& operator+=(const Vector3& other);
    Vector3& operator-=(const Vector3& other);
    Vector3& operator*=(float scalar);
    Vector3& operator*=(const Vector3& other);
    Vector3& operator/=(float scalar);
    Vector3& operator/=(const Vector3& other);

    constexpr Vector3 operator-() const { return Vector3(-x, -y, -z); }
    constexpr bool operator==(const Vector3& other) const {
        return Math::abs(x - other.x) < Math::EPSILON && 
               Math::abs(y - other.y) < Math::EPSILON && 
               Math::abs(z - other.z) < Math::EPSILON;
    }
    constexpr bool operator!=(const Vector3& other) const { return !(*this == other); }
    
    // 浮点数比较，不能直接==，这个要注意
    bool isApproximatelyEqual(const Vector3& other, float epsilon = Math::EPSILON) const;

    // 长度相关
    [[nodiscard]] constexpr float length() const { return Math::sqrt(lengthSquared()); }
    [[nodiscard]] constexpr float magnitude() const { return length(); }
    [[nodiscard]] constexpr float lengthSquared() const { return x * x + y * y + z * z; }
    [[nodiscard]] constexpr float magnitudeSquared() const { return lengthSquared(); }
    [[nodiscard]] constexpr float distance(const Vector3& other) const { return (*this - other).length(); }
    [[nodiscard]] constexpr float distanceSquared(const Vector3& other) const { return (*this - other).lengthSquared(); }
    // fastLength用fastInvSqrt算的，快是快，精度差一点
    [[nodiscard]] constexpr float fastLength() const { 
        float lenSq = lengthSquared();
        return lenSq > 0.0f ? lenSq * Math::fastInvSqrt(lenSq) : 0.0f;
    }

    // 归一化
    Vector3& normalize();
    [[nodiscard]] inline Vector3 normalized() const {
        float len = std::sqrt(lengthSquared());
        if (len < Math::EPSILON) {
            return Vector3::zero;
        }
        return *this * (1.0f / len);
    }
    // 快速归一化
    [[nodiscard]] constexpr Vector3 fastNormalized() const {
        float lenSq = lengthSquared();
        if (lenSq < Math::EPSILON * Math::EPSILON) {
            return Vector3::zero;
        }
        return *this * Math::fastInvSqrt(lenSq);
    }
    [[nodiscard]] inline Vector3 unit() const { return normalized(); }

    // 点积
    [[nodiscard]] constexpr float dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    // 叉积，返回垂直于两个向量的向量
    [[nodiscard]] constexpr Vector3 cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // 夹角
    [[nodiscard]] constexpr float angle(const Vector3& other) const {
        float dotProduct = dot(other);
        float lenProduct = length() * other.length();
        
        if (lenProduct < Math::EPSILON) {
            return 0.0f;
        }
        
        float cosAngle = Math::clamp(dotProduct / lenProduct, -1.0f, 1.0f);
        return Math::acos(cosAngle);
    }
    // 角度制
    [[nodiscard]] constexpr float angleDegrees(const Vector3& other) const { return Math::radToDeg(angle(other)); }

    // 投影到另一个向量上
    [[nodiscard]] constexpr Vector3 project(const Vector3& other) const {
        float otherLenSq = other.lengthSquared();
        if (otherLenSq < Math::EPSILON) {
            return Vector3::zero;
        }
        float scale = dot(other) / otherLenSq;
        return other * scale;
    }
    // 反射，normal得是单位向量才行
    [[nodiscard]] constexpr Vector3 reflect(const Vector3& normal) const {
        float dotProduct = dot(normal);
        return *this - normal * (2.0f * dotProduct);
    }
    // 折射，eta是折射率比值
    [[nodiscard]] constexpr Vector3 refract(const Vector3& normal, float eta) const {
        float dotProduct = dot(normal);
        float k = 1.0f - eta * eta * (1.0f - dotProduct * dotProduct);
        if (k < 0.0f) return Vector3::zero;
        float sqrtK = Math::sqrt(k);
        return *this * eta + normal * (eta * dotProduct - sqrtK);
    }

    // lerp，游戏里用得最多
    [[nodiscard]] constexpr Vector3 lerp(const Vector3& other, float t) const {
        float clampedT = (t < 0.0f) ? 0.0f : ((t > 1.0f) ? 1.0f : t);
        return *this + (other - *this) * clampedT;
    }
    // 不clamp的版本
    [[nodiscard]] constexpr Vector3 fastLerp(const Vector3& other, float t) const {
        return *this * (1.0f - t) + other * t;
    }
    // slerp，旋转用这个更平滑，不过计算量大一点
    [[nodiscard]] constexpr Vector3 slerp(const Vector3& other, float t) const {
        float dotProduct = dot(other);
        float lenProduct = length() * other.length();
        if (lenProduct < Math::EPSILON) return lerp(other, t);
        float cosAngle = Math::clamp(dotProduct / lenProduct, -1.0f, 1.0f);
        float angle = Math::acos(cosAngle);
        float sinAngle = Math::sin(angle);
        if (Math::abs(sinAngle) < Math::EPSILON) return lerp(other, t);
        float t1 = Math::sin((1.0f - t) * angle) / sinAngle;
        float t2 = Math::sin(t * angle) / sinAngle;
        return *this * t1 + other * t2;
    }

    // clamp
    [[nodiscard]] constexpr Vector3 clamp(const Vector3& min, const Vector3& max) const {
        return Vector3(
            (x < min.x) ? min.x : ((x > max.x) ? max.x : x),
            (y < min.y) ? min.y : ((y > max.y) ? max.y : y),
            (z < min.z) ? min.z : ((z > max.z) ? max.z : z)
        );
    }

    static const Vector3 zero;
    static const Vector3 one;
    static const Vector3 right;
    static const Vector3 left;
    static const Vector3 up;
    static const Vector3 down;
    static const Vector3 forward;
    static const Vector3 back;

    // 静态方法
    [[nodiscard]] static constexpr Vector3 min(const Vector3& a, const Vector3& b) {
        return Vector3((a.x < b.x) ? a.x : b.x, (a.y < b.y) ? a.y : b.y, (a.z < b.z) ? a.z : b.z);
    }
    [[nodiscard]] static constexpr Vector3 max(const Vector3& a, const Vector3& b) {
        return Vector3((a.x > b.x) ? a.x : b.x, (a.y > b.y) ? a.y : b.y, (a.z > b.z) ? a.z : b.z);
    }
    [[nodiscard]] static constexpr Vector3 abs(const Vector3& v) {
        return Vector3(v.x < 0 ? -v.x : v.x, v.y < 0 ? -v.y : v.y, v.z < 0 ? -v.z : v.z);
    }
    [[nodiscard]] static constexpr Vector3 floor(const Vector3& v) {
        return Vector3(Math::floor(v.x), Math::floor(v.y), Math::floor(v.z));
    }
    [[nodiscard]] static constexpr Vector3 ceil(const Vector3& v) {
        return Vector3(Math::ceil(v.x), Math::ceil(v.y), Math::ceil(v.z));
    }
    [[nodiscard]] static constexpr Vector3 round(const Vector3& v) {
        return Vector3(Math::round(v.x), Math::round(v.y), Math::round(v.z));
    }
    [[nodiscard]] static inline Vector3 normalize(const Vector3& v) { return v.normalized(); }
    [[nodiscard]] static constexpr float dot(const Vector3& a, const Vector3& b) { return a.dot(b); }
    [[nodiscard]] static constexpr Vector3 cross(const Vector3& a, const Vector3& b) { return a.cross(b); }
    [[nodiscard]] static inline float angle(const Vector3& a, const Vector3& b) { return a.angle(b); }
    [[nodiscard]] static constexpr float distance(const Vector3& a, const Vector3& b) { return a.distance(b); }
    [[nodiscard]] static constexpr Vector3 lerp(const Vector3& a, const Vector3& b, float t) { return a.lerp(b, t); }
    [[nodiscard]] static inline Vector3 slerp(const Vector3& a, const Vector3& b, float t) { return a.slerp(b, t); }
    
    // 球坐标转xyz
    [[nodiscard]] static constexpr Vector3 fromSpherical(float radius, float theta, float phi) {
        float sinPhi = Math::sin(phi);
        return Vector3(radius * sinPhi * Math::sin(theta), radius * Math::cos(phi), radius * sinPhi * Math::cos(theta));
    }
    // 柱坐标
    [[nodiscard]] static constexpr Vector3 fromCylindrical(float radius, float theta, float height) {
        return Vector3(radius * Math::sin(theta), height, radius * Math::cos(theta));
    }
    [[nodiscard]] static Vector3 random(float min = 0.0f, float max = 1.0f);
    [[nodiscard]] static Vector3 randomDirection();
    
    // 其他
    [[nodiscard]] constexpr bool isZero() const {
        return Math::approximatelyEqual(x, 0.0f) && Math::approximatelyEqual(y, 0.0f) && Math::approximatelyEqual(z, 0.0f);
    }
    [[nodiscard]] constexpr bool isUnit() const { return Math::approximatelyEqual(lengthSquared(), 1.0f); }
    // 找个垂直向量
    [[nodiscard]] inline Vector3 perpendicular() const {
        if (Math::abs(x) < Math::abs(y)) {
            return Vector3(-z, 0.0f, x).normalized();
        } else {
            return Vector3(0.0f, z, -y).normalized();
        }
    }
    // 绕轴旋转，罗德里格斯公式
    [[nodiscard]] inline Vector3 rotate(float angle, const Vector3& axis) const {
        float cosAngle = Math::cos(angle);
        float sinAngle = Math::sin(angle);
        Vector3 normAxis = axis.normalized();
        return (*this * cosAngle) + (normAxis.cross(*this) * sinAngle) + (normAxis * normAxis.dot(*this) * (1.0f - cosAngle));
    }
    // 投影到平面
    [[nodiscard]] constexpr Vector3 projectOnPlane(const Vector3& planeNormal) const {
        return *this - project(planeNormal);
    }
    // reject，去掉在另一个向量上的投影
    [[nodiscard]] constexpr Vector3 reject(const Vector3& other) const {
        return *this - project(other);
    }
    // 中点
    [[nodiscard]] constexpr Vector3 midpoint(const Vector3& other) const {
        return (*this + other) * 0.5f;
    }
    // 缩放
    [[nodiscard]] constexpr Vector3 scale(const Vector3& scale) const {
        return Vector3(x * scale.x, y * scale.y, z * scale.z);
    }
    [[nodiscard]] constexpr float minComponent() const {
        return (x < y) ? ((x < z) ? x : z) : ((y < z) ? y : z);
    }
    [[nodiscard]] constexpr float maxComponent() const {
        return (x > y) ? ((x > z) ? x : z) : ((y > z) ? y : z);
    }
    [[nodiscard]] constexpr int minAxis() const {
        return (x < y) ? ((x < z) ? 0 : 2) : ((y < z) ? 1 : 2);
    }
    [[nodiscard]] constexpr int maxAxis() const {
        return (x > y) ? ((x > z) ? 0 : 2) : ((y > z) ? 1 : 2);
    }
    // 限制长度
    [[nodiscard]] constexpr Vector3 clampLength(float minLength, float maxLength) const {
        float lenSq = lengthSquared();
        if (lenSq < minLength * minLength) {
            float scale = minLength / Math::sqrt(lenSq);
            return *this * scale;
        }
        if (lenSq > maxLength * maxLength) {
            float scale = maxLength / Math::sqrt(lenSq);
            return *this * scale;
        }
        return *this;
    }
    // 朝目标移动
    [[nodiscard]] constexpr Vector3 moveTowards(const Vector3& target, float maxDistanceDelta) const {
        Vector3 diff = *this - target;
        float distSq = diff.lengthSquared();
        if (distSq == 0.0f || maxDistanceDelta >= Math::sqrt(distSq)) {
            return target;
        }
        return *this - diff * (maxDistanceDelta / Math::sqrt(distSq));
    }
    // 矩阵变换
    [[nodiscard]] Vector3 transform(const class Matrix4x4& matrix) const;
    
    std::string toString() const;
    
    friend std::ostream& operator<<(std::ostream& os, const Vector3& v);
};

constexpr Vector3 operator*(float scalar, const Vector3& vector) { return vector * scalar; }
constexpr Vector3 operator+(float scalar, const Vector3& vector) {
    return Vector3(scalar + vector.x, scalar + vector.y, scalar + vector.z);
}
constexpr Vector3 operator+(const Vector3& vector, float scalar) {
    return Vector3(vector.x + scalar, vector.y + scalar, vector.z + scalar);
}
constexpr Vector3 operator-(const Vector3& vector, float scalar) {
    return Vector3(vector.x - scalar, vector.y - scalar, vector.z - scalar);
}
constexpr Vector3 operator/(float scalar, const Vector3& vector) {
    return Vector3(scalar / vector.x, scalar / vector.y, scalar / vector.z);
}

// fastInvSqrt的魔法数字 0x5f3759df，Quake3里来的

} // namespace AstraAlgebra
