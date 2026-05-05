#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <iostream>

namespace AstraAlgebra {

// 双精度3D向量，高精度场景用的，一般游戏float就够了，除非你要做航天级计算（不是
class Vector3d {
public:
    // 用union是为了后面和SIMD对齐方便，虽然double的SIMD用得少
    union {
        struct {
            double x, y, z;
        };
        // 凑够16字节对齐，虽然double一般用不到SIMD
        double data[4];
    };

    // 设置xyz，简单粗暴
    void set(double x, double y, double z) {
        this->x = x;
        this->y = y;
        this->z = z;
    }

    // 构造函数，有好几种
    constexpr Vector3d() : x(0.0), y(0.0), z(0.0) {}
    constexpr Vector3d(double x, double y, double z) : x(x), y(y), z(z) {}
    constexpr Vector3d(double scalar) : x(scalar), y(scalar), z(scalar) {}
    constexpr Vector3d(const Vector3d& other) : x(other.x), y(other.y), z(other.z) {}

    // 析构函数，默认的就行
    ~Vector3d() = default;

    // 赋值，就那样
    Vector3d& operator=(const Vector3d& other) {
        if (this != &other) {
            x = other.x;
            y = other.y;
            z = other.z;
        }
        return *this;
    }

    // 用[]访问分量，方便
    constexpr double& operator[](int index) { return (&x)[index]; }
    constexpr const double& operator[](int index) const { return (&x)[index]; }

    // 加减乘除，最基本的
    constexpr Vector3d operator+(const Vector3d& other) const {
        return Vector3d(x + other.x, y + other.y, z + other.z);
    }
    constexpr Vector3d operator-(const Vector3d& other) const {
        return Vector3d(x - other.x, y - other.y, z - other.z);
    }
    constexpr Vector3d operator*(double scalar) const {
        return Vector3d(x * scalar, y * scalar, z * scalar);
    }
    constexpr Vector3d operator*(const Vector3d& other) const {
        return Vector3d(x * other.x, y * other.y, z * other.z);
    }
    constexpr Vector3d operator/(double scalar) const {
        double invScalar = 1.0 / scalar;
        return (*this) * invScalar;
    }
    constexpr Vector3d operator/(const Vector3d& other) const {
        return Vector3d(x / other.x, y / other.y, z / other.z);
    }

    // 复合赋值，+= -= 那些
    Vector3d& operator+=(const Vector3d& other) {
        x += other.x; y += other.y; z += other.z;
        return *this;
    }
    Vector3d& operator-=(const Vector3d& other) {
        x -= other.x; y -= other.y; z -= other.z;
        return *this;
    }
    Vector3d& operator*=(double scalar) {
        x *= scalar; y *= scalar; z *= scalar;
        return *this;
    }
    Vector3d& operator*=(const Vector3d& other) {
        x *= other.x; y *= other.y; z *= other.z;
        return *this;
    }
    Vector3d& operator/=(double scalar) {
        double inv = 1.0 / scalar;
        x *= inv; y *= inv; z *= inv;
        return *this;
    }
    Vector3d& operator/=(const Vector3d& other) {
        x /= other.x; y /= other.y; z /= other.z;
        return *this;
    }

    // 取反和比较
    constexpr Vector3d operator-() const { return Vector3d(-x, -y, -z); }
    constexpr bool operator==(const Vector3d& other) const {
        return Math::abs(x - other.x) < Math::EPSILON_D && 
               Math::abs(y - other.y) < Math::EPSILON_D && 
               Math::abs(z - other.z) < Math::EPSILON_D;
    }
    constexpr bool operator!=(const Vector3d& other) const { return !(*this == other); }

    // 长度相关
    constexpr double lengthSquared() const { return x * x + y * y + z * z; }
    constexpr double length() const { return Math::sqrt(lengthSquared()); }
    constexpr double distance(const Vector3d& other) const { return (*this - other).length(); }
    constexpr double distanceSquared(const Vector3d& other) const { return (*this - other).lengthSquared(); }

    // 归一化
    constexpr Vector3d normalized() const {
        double len = Math::sqrt(lengthSquared());
        return (len > Math::EPSILON_D) ? Vector3d(x / len, y / len, z / len) : Vector3d::zero;
    }
    Vector3d& normalize() {
        double len = length();
        if (len > Math::EPSILON_D) {
            double inv = 1.0 / len;
            x *= inv; y *= inv; z *= inv;
        }
        return *this;
    }

    // 点积
    constexpr double dot(const Vector3d& other) const {
        return x * other.x + y * other.y + z * other.z;
    }
    // 叉积，返回垂直于两个向量的向量
    constexpr Vector3d cross(const Vector3d& other) const {
        return Vector3d(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    // lerp，游戏里用得最多
    constexpr Vector3d lerp(const Vector3d& other, double t) const {
        return Vector3d(
            Math::lerp(x, other.x, t),
            Math::lerp(y, other.y, t),
            Math::lerp(z, other.z, t)
        );
    }

    // 夹角，可以返回弧度或角度
    double angle(const Vector3d& other, bool returnDegrees = false) const {
        double dotProduct = dot(other);
        double lengths = length() * other.length();
        if (lengths < Math::EPSILON_D) return 0.0;
        double cosAngle = Math::clamp(dotProduct / lengths, -1.0, 1.0);
        double angle = Math::acos(cosAngle);
        return returnDegrees ? Math::radToDeg(angle) : angle;
    }

    // 反射，normal得是单位向量
    constexpr Vector3d reflect(const Vector3d& normal) const {
        return *this - normal * (2.0 * dot(normal));
    }

    // 投影到另一个向量上
    constexpr Vector3d project(const Vector3d& onto) const {
        double dotProduct = dot(onto);
        double lenSq = onto.lengthSquared();
        if (lenSq < Math::EPSILON_D) return Vector3d::zero;
        return onto * (dotProduct / lenSq);
    }

    // 常用的常量，直接用就行
    static const Vector3d zero;
    static const Vector3d one;
    static const Vector3d up;
    static const Vector3d down;
    static const Vector3d forward;
    static const Vector3d back;
    static const Vector3d right;
    static const Vector3d left;

    // 静态方法
    static constexpr Vector3d Zero() { return Vector3d(0.0, 0.0, 0.0); }
    static constexpr Vector3d One() { return Vector3d(1.0, 1.0, 1.0); }
    static constexpr Vector3d Up() { return Vector3d(0.0, 1.0, 0.0); }
    static constexpr Vector3d Forward() { return Vector3d(0.0, 0.0, 1.0); }
    static constexpr Vector3d Right() { return Vector3d(1.0, 0.0, 0.0); }

    static constexpr double dot(const Vector3d& a, const Vector3d& b) { return a.dot(b); }
    static constexpr Vector3d cross(const Vector3d& a, const Vector3d& b) { return a.cross(b); }
    static constexpr double distance(const Vector3d& a, const Vector3d& b) { return a.distance(b); }
    static constexpr Vector3d lerp(const Vector3d& a, const Vector3d& b, double t) { return a.lerp(b, t); }

    // 输出，调试用
    friend std::ostream& operator<<(std::ostream& os, const Vector3d& v) {
        os << "Vector3d(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
};

// 静态常量初始化
inline const Vector3d Vector3d::zero(0.0, 0.0, 0.0);
inline const Vector3d Vector3d::one(1.0, 1.0, 1.0);
inline const Vector3d Vector3d::up(0.0, 1.0, 0.0);
inline const Vector3d Vector3d::down(0.0, -1.0, 0.0);
inline const Vector3d Vector3d::forward(0.0, 0.0, 1.0);
inline const Vector3d Vector3d::back(0.0, 0.0, -1.0);
inline const Vector3d Vector3d::right(1.0, 0.0, 0.0);
inline const Vector3d Vector3d::left(-1.0, 0.0, 0.0);

// 全局操作符，让 scalar * vector 也能用
constexpr Vector3d operator*(double scalar, const Vector3d& v) { return v * scalar; }
constexpr Vector3d operator+(double scalar, const Vector3d& v) {
    return Vector3d(scalar + v.x, scalar + v.y, scalar + v.z);
}
constexpr Vector3d operator+(const Vector3d& v, double scalar) {
    return Vector3d(v.x + scalar, v.y + scalar, v.z + scalar);
}
constexpr Vector3d operator-(const Vector3d& v, double scalar) {
    return Vector3d(v.x - scalar, v.y - scalar, v.z - scalar);
}
constexpr Vector3d operator/(double scalar, const Vector3d& v) {
    return Vector3d(scalar / v.x, scalar / v.y, scalar / v.z);
}

} // namespace AstraAlgebra
