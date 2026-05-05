// Vector2.cpp - 2D向量类实现
// 2D场景、UI系统经常用

#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

namespace AstraAlgebra {

// 初始化静态常量
const Vector2 Vector2::zero(0.0f, 0.0f);
const Vector2 Vector2::one(1.0f, 1.0f);
const Vector2 Vector2::right(1.0f, 0.0f);
const Vector2 Vector2::left(-1.0f, 0.0f);
const Vector2 Vector2::up(0.0f, 1.0f);
const Vector2 Vector2::down(0.0f, -1.0f);

// 赋值操作符
Vector2& Vector2::operator=(const Vector2& other) {
    if (this != &other) {
        x = other.x;
        y = other.y;
    }
    return *this;
}

Vector2& Vector2::operator=(Vector2&& other) noexcept {
    if (this != &other) {
        x = other.x;
        y = other.y;
        other.x = 0.0f;
        other.y = 0.0f;
    }
    return *this;
}

// 复合赋值操作符
Vector2& Vector2::operator+=(const Vector2& other) {
    x += other.x;
    y += other.y;
    return *this;
}

Vector2& Vector2::operator-=(const Vector2& other) {
    x -= other.x;
    y -= other.y;
    return *this;
}

Vector2& Vector2::operator*=(float scalar) {
    x *= scalar;
    y *= scalar;
    return *this;
}

Vector2& Vector2::operator/=(float scalar) {
    if (std::abs(scalar) > Math::EPSILON) {
        x /= scalar;
        y /= scalar;
    }
    return *this;
}

// 向量归一化
Vector2& Vector2::normalize() {
    float len = length();
    if (len > Math::EPSILON) {
        x /= len;
        y /= len;
    }
    return *this;
}

// 向量投影
Vector2 Vector2::project(const Vector2& other) const {
    float otherLenSq = other.lengthSquared();
    if (otherLenSq < Math::EPSILON) {
        return Vector2::zero;
    }
    
    float scale = dot(other) / otherLenSq;
    return other * scale;
}

// 向量反射
Vector2 Vector2::reflect(const Vector2& normal) const {
    float dotProduct = dot(normal);
    return *this - normal * (2.0f * dotProduct);
}

// 调试输出
std::ostream& operator<<(std::ostream& os, const Vector2& v) {
    os << "(" << v.x << ", " << v.y << ")";
    return os;
}

} // namespace AstraAlgebra