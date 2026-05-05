// Vector4 - 4D向量，齐次坐标、颜色这些会用到

#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <iostream>

namespace AstraAlgebra {

// 4D向量，xyzw，齐次坐标和颜色会用到
class Vector4 {
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
    
    void set(float x, float y, float z, float w) {
        this->x = x;
        this->y = y;
        this->z = z;
        this->w = w;
    }

    constexpr Vector4() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
    constexpr Vector4(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    constexpr Vector4(float scalar) : x(scalar), y(scalar), z(scalar), w(scalar) {}
    constexpr Vector4(const Vector4& other) : x(other.x), y(other.y), z(other.z), w(other.w) {}
    constexpr Vector4(Vector4&& other) noexcept : x(other.x), y(other.y), z(other.z), w(other.w) {
        other.x = 0.0f; other.y = 0.0f; other.z = 0.0f; other.w = 0.0f;
    }
    constexpr Vector4(const Vector2& v2, float z = 0.0f, float w = 1.0f) : x(v2.x), y(v2.y), z(z), w(w) {}
    constexpr Vector4(const Vector3& v3, float w = 1.0f) : x(v3.x), y(v3.y), z(v3.z), w(w) {}

    ~Vector4() = default;

    Vector4& operator=(const Vector4& other);
    Vector4& operator=(Vector4&& other) noexcept;
    Vector4& operator=(const Vector2& other);
    Vector4& operator=(const Vector3& other);

    constexpr float& operator[](int index) { return (&x)[index]; }
    constexpr const float& operator[](int index) const { return (&x)[index]; }

    constexpr Vector4 operator+(const Vector4& other) const {
        return Vector4(x + other.x, y + other.y, z + other.z, w + other.w);
    }
    constexpr Vector4 operator-(const Vector4& other) const {
        return Vector4(x - other.x, y - other.y, z - other.z, w - other.w);
    }
    constexpr Vector4 operator*(float scalar) const {
        return Vector4(x * scalar, y * scalar, z * scalar, w * scalar);
    }
    constexpr Vector4 operator/(float scalar) const {
        float invScalar = 1.0f / scalar;
        return Vector4(x * invScalar, y * invScalar, z * invScalar, w * invScalar);
    }

    Vector4& operator+=(const Vector4& other);
    Vector4& operator-=(const Vector4& other);
    Vector4& operator*=(float scalar);
    Vector4& operator/=(float scalar);

    constexpr Vector4 operator-() const { return Vector4(-x, -y, -z, -w); }
    constexpr bool operator==(const Vector4& other) const {
        return Math::abs(x - other.x) < Math::EPSILON && 
               Math::abs(y - other.y) < Math::EPSILON && 
               Math::abs(z - other.z) < Math::EPSILON &&
               Math::abs(w - other.w) < Math::EPSILON;
    }
    constexpr bool operator!=(const Vector4& other) const { return !(*this == other); }

    // 长度
    constexpr float length() const { return Math::sqrt(lengthSquared()); }
    constexpr float magnitude() const { return length(); }
    constexpr float lengthSquared() const { return x * x + y * y + z * z + w * w; }
    constexpr float magnitudeSquared() const { return lengthSquared(); }
    constexpr float distance(const Vector4& other) const { return (*this - other).length(); }
    constexpr float distanceSquared(const Vector4& other) const { return (*this - other).lengthSquared(); }

    // 归一化
    Vector4& normalize();
    constexpr Vector4 normalized() const {
        float len = length();
        return (len > Math::EPSILON) ? *this * (1.0f / len) : *this;
    }
    constexpr Vector4 unit() const { return normalized(); }

    // 点积
    constexpr float dot(const Vector4& other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }

    // 夹角
    constexpr float angle(const Vector4& other) const {
        float dotProduct = dot(other);
        float lenProduct = length() * other.length();
        if (lenProduct < Math::EPSILON) return 0.0f;
        float cosAngle = Math::clamp(dotProduct / lenProduct, -1.0f, 1.0f);
        return Math::acos(cosAngle);
    }
    constexpr float angleDegrees(const Vector4& other) const { return Math::radToDeg(angle(other)); }

    // 投影
    Vector4 project(const Vector4& other) const;
    Vector4 reflect(const Vector4& normal) const;

    // 插值
    constexpr Vector4 lerp(const Vector4& other, float t) const {
        float clampedT = (t < 0.0f) ? 0.0f : ((t > 1.0f) ? 1.0f : t);
        return *this + (other - *this) * clampedT;
    }
    Vector4 slerp(const Vector4& other, float t) const;

    // clamp
    constexpr Vector4 clamp(const Vector4& min, const Vector4& max) const {
        return Vector4(
            (x < min.x) ? min.x : ((x > max.x) ? max.x : x),
            (y < min.y) ? min.y : ((y > max.y) ? max.y : y),
            (z < min.z) ? min.z : ((z > max.z) ? max.z : z),
            (w < min.w) ? min.w : ((w > max.w) ? max.w : w)
        );
    }

    constexpr Vector2 toVector2() const { return Vector2(x, y); }
    constexpr Vector3 toVector3() const { return Vector3(x, y, z); }

    static const Vector4 zero;
    static const Vector4 one;
    static const Vector4 right;
    static const Vector4 left;
    static const Vector4 up;
    static const Vector4 down;
    static const Vector4 forward;
    static const Vector4 back;
    static const Vector4 oneW;

    // 静态方法
    static constexpr Vector4 min(const Vector4& a, const Vector4& b) {
        return Vector4(
            (a.x < b.x) ? a.x : b.x,
            (a.y < b.y) ? a.y : b.y,
            (a.z < b.z) ? a.z : b.z,
            (a.w < b.w) ? a.w : b.w
        );
    }
    static constexpr Vector4 max(const Vector4& a, const Vector4& b) {
        return Vector4(
            (a.x > b.x) ? a.x : b.x,
            (a.y > b.y) ? a.y : b.y,
            (a.z > b.z) ? a.z : b.z,
            (a.w > b.w) ? a.w : b.w
        );
    }
    static constexpr Vector4 abs(const Vector4& v) {
        return Vector4(
            v.x < 0 ? -v.x : v.x,
            v.y < 0 ? -v.y : v.y,
            v.z < 0 ? -v.z : v.z,
            v.w < 0 ? -v.w : v.w
        );
    }
    static constexpr Vector4 floor(const Vector4& v) {
        return Vector4(Math::floor(v.x), Math::floor(v.y), Math::floor(v.z), Math::floor(v.w));
    }
    static constexpr Vector4 ceil(const Vector4& v) {
        return Vector4(Math::ceil(v.x), Math::ceil(v.y), Math::ceil(v.z), Math::ceil(v.w));
    }
    static constexpr Vector4 round(const Vector4& v) {
        return Vector4(Math::round(v.x), Math::round(v.y), Math::round(v.z), Math::round(v.w));
    }
    static Vector4 normalize(const Vector4& v);
    static constexpr float dot(const Vector4& a, const Vector4& b) { return a.dot(b); }
    static constexpr float angle(const Vector4& a, const Vector4& b) { return a.angle(b); }
    static constexpr float distance(const Vector4& a, const Vector4& b) { return a.distance(b); }
    static constexpr Vector4 lerp(const Vector4& a, const Vector4& b, float t) { return a.lerp(b, t); }
    static Vector4 slerp(const Vector4& a, const Vector4& b, float t);

    friend std::ostream& operator<<(std::ostream& os, const Vector4& v);
};

constexpr Vector4 operator*(float scalar, const Vector4& vector) { return vector * scalar; }
constexpr Vector4 operator+(float scalar, const Vector4& vector) {
    return Vector4(scalar + vector.x, scalar + vector.y, scalar + vector.z, scalar + vector.w);
}
constexpr Vector4 operator+(const Vector4& vector, float scalar) {
    return Vector4(vector.x + scalar, vector.y + scalar, vector.z + scalar, vector.w + scalar);
}
constexpr Vector4 operator-(const Vector4& vector, float scalar) {
    return Vector4(vector.x - scalar, vector.y - scalar, vector.z - scalar, vector.w - scalar);
}
constexpr Vector4 operator/(float scalar, const Vector4& vector) {
    return Vector4(scalar / vector.x, scalar / vector.y, scalar / vector.z, scalar / vector.w);
}

} // namespace AstraAlgebra
