// Vector2 - 2D向量

#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <iostream>

namespace AstraAlgebra {

// 2D向量，xy平面用的
class Vector2 {
public:
    float x, y;

    constexpr Vector2() : x(0.0f), y(0.0f) {}
    constexpr Vector2(float x, float y) : x(x), y(y) {}
    constexpr Vector2(float scalar) : x(scalar), y(scalar) {}
    constexpr Vector2(const Vector2& other) : x(other.x), y(other.y) {}
    constexpr Vector2(Vector2&& other) noexcept : x(other.x), y(other.y) {
        other.x = 0.0f;
        other.y = 0.0f;
    }

    ~Vector2() = default;

    Vector2& operator=(const Vector2& other);
    Vector2& operator=(Vector2&& other) noexcept;

    constexpr float& operator[](int index) { return (&x)[index]; }
    constexpr const float& operator[](int index) const { return (&x)[index]; }

    constexpr Vector2 operator+(const Vector2& other) const { return Vector2(x + other.x, y + other.y); }
    constexpr Vector2 operator-(const Vector2& other) const { return Vector2(x - other.x, y - other.y); }
    constexpr Vector2 operator*(float scalar) const { return Vector2(x * scalar, y * scalar); }
    constexpr Vector2 operator/(float scalar) const {
        return (scalar > Math::EPSILON || scalar < -Math::EPSILON) ? Vector2(x / scalar, y / scalar) : Vector2::zero;
    }

    Vector2& operator+=(const Vector2& other);
    Vector2& operator-=(const Vector2& other);
    Vector2& operator*=(float scalar);
    Vector2& operator/=(float scalar);

    constexpr Vector2 operator-() const { return Vector2(-x, -y); }
    constexpr bool operator==(const Vector2& other) const {
        return (x - other.x < Math::EPSILON && x - other.x > -Math::EPSILON) && 
               (y - other.y < Math::EPSILON && y - other.y > -Math::EPSILON);
    }
    constexpr bool operator!=(const Vector2& other) const { return !(*this == other); }

    // 长度
    constexpr float length() const { return Math::sqrt(lengthSquared()); }
    constexpr float magnitude() const { return length(); }
    constexpr float lengthSquared() const { return x * x + y * y; }
    constexpr float magnitudeSquared() const { return lengthSquared(); }
    constexpr float distance(const Vector2& other) const { return (*this - other).length(); }
    constexpr float distanceSquared(const Vector2& other) const { return (*this - other).lengthSquared(); }

    // 归一化
    Vector2& normalize();
    constexpr Vector2 normalized() const {
        float len = length();
        return (len > Math::EPSILON) ? Vector2(x / len, y / len) : *this;
    }
    constexpr Vector2 unit() const { return normalized(); }

    // 点积
    constexpr float dot(const Vector2& other) const { return x * other.x + y * other.y; }

    // 叉积，2D叉积返回标量
    constexpr float cross(const Vector2& other) const { return x * other.y - y * other.x; }

    // 夹角
    constexpr float angle(const Vector2& other) const {
        float dotProduct = dot(other);
        float lenProduct = length() * other.length();
        if (lenProduct < Math::EPSILON) return 0.0f;
        float cosAngle = dotProduct / lenProduct;
        cosAngle = (cosAngle > 1.0f) ? 1.0f : ((cosAngle < -1.0f) ? -1.0f : cosAngle);
        return Math::acos(cosAngle);
    }
    constexpr float angleDegrees(const Vector2& other) const { return angle(other) * (180.0f / Math::PI); }

    // 投影
    Vector2 project(const Vector2& other) const;
    Vector2 reflect(const Vector2& normal) const;

    // 插值
    constexpr Vector2 lerp(const Vector2& other, float t) const {
        float clampedT = (t < 0.0f) ? 0.0f : ((t > 1.0f) ? 1.0f : t);
        return *this + (other - *this) * clampedT;
    }
    constexpr Vector2 slerp(const Vector2& other, float t) const { return lerp(other, t); }

    // clamp
    constexpr Vector2 clamp(const Vector2& min, const Vector2& max) const {
        return Vector2(
            (x < min.x) ? min.x : ((x > max.x) ? max.x : x),
            (y < min.y) ? min.y : ((y > max.y) ? max.y : y)
        );
    }

    static const Vector2 zero;
    static const Vector2 one;
    static const Vector2 right;
    static const Vector2 left;
    static const Vector2 up;
    static const Vector2 down;

    // 静态方法
    static constexpr Vector2 min(const Vector2& a, const Vector2& b) { return Vector2((a.x < b.x) ? a.x : b.x, (a.y < b.y) ? a.y : b.y); }
    static constexpr Vector2 max(const Vector2& a, const Vector2& b) { return Vector2((a.x > b.x) ? a.x : b.x, (a.y > b.y) ? a.y : b.y); }
    static constexpr Vector2 abs(const Vector2& v) { return Vector2(v.x < 0 ? -v.x : v.x, v.y < 0 ? -v.y : v.y); }
    static constexpr Vector2 floor(const Vector2& v) { return Vector2(Math::floor(v.x), Math::floor(v.y)); }
    static constexpr Vector2 ceil(const Vector2& v) { return Vector2(Math::ceil(v.x), Math::ceil(v.y)); }
    static constexpr Vector2 round(const Vector2& v) { return Vector2(Math::round(v.x), Math::round(v.y)); }
    static constexpr Vector2 normalize(const Vector2& v) { return v.normalized(); }
    static constexpr float dot(const Vector2& a, const Vector2& b) { return a.dot(b); }
    static constexpr float cross(const Vector2& a, const Vector2& b) { return a.cross(b); }
    static constexpr float angle(const Vector2& a, const Vector2& b) { return a.angle(b); }
    static constexpr float distance(const Vector2& a, const Vector2& b) { return a.distance(b); }
    static constexpr Vector2 lerp(const Vector2& a, const Vector2& b, float t) { return a.lerp(b, t); }
    static constexpr Vector2 slerp(const Vector2& a, const Vector2& b, float t) { return a.slerp(b, t); }

    friend std::ostream& operator<<(std::ostream& os, const Vector2& v);
};

constexpr Vector2 operator*(float scalar, const Vector2& vector) { return vector * scalar; }
constexpr Vector2 operator+(float scalar, const Vector2& vector) { return Vector2(scalar + vector.x, scalar + vector.y); }
constexpr Vector2 operator+(const Vector2& vector, float scalar) { return Vector2(vector.x + scalar, vector.y + scalar); }
constexpr Vector2 operator-(const Vector2& vector, float scalar) { return Vector2(vector.x - scalar, vector.y - scalar); }
constexpr Vector2 operator/(float scalar, const Vector2& vector) { return Vector2(scalar / vector.x, scalar / vector.y); }

} // namespace AstraAlgebra
