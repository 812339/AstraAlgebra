#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Vector/Vector2.h>
#include <iostream>

namespace AstraAlgebra {

// 2x2矩阵，2D变换用的，旋转缩放那些
class Matrix2x2 {
public:
    // 数据就存在这个二维数组里，行优先
    float m[2][2];

    // 构造函数，有好几种
    constexpr Matrix2x2() : m{{0.0f, 0.0f}, {0.0f, 0.0f}} {}
    constexpr Matrix2x2(float m00, float m01, float m10, float m11) : m{{m00, m01}, {m10, m11}} {}
    // 标量构造，对角线是这个值，其他是0
    constexpr Matrix2x2(float scalar) : m{{scalar, 0.0f}, {0.0f, scalar}} {}
    constexpr Matrix2x2(const Matrix2x2& other) : m{{other.m[0][0], other.m[0][1]}, {other.m[1][0], other.m[1][1]}} {}
    Matrix2x2(Matrix2x2&& other) noexcept {
        m[0][0] = other.m[0][0]; m[0][1] = other.m[0][1];
        m[1][0] = other.m[1][0]; m[1][1] = other.m[1][1];
        other.m[0][0] = 0.0f; other.m[0][1] = 0.0f;
        other.m[1][0] = 0.0f; other.m[1][1] = 0.0f;
    }
    // 用两个向量构造，每个向量是一行
    constexpr Matrix2x2(const Vector2& v0, const Vector2& v1) : m{{v0.x, v0.y}, {v1.x, v1.y}} {}
    constexpr Matrix2x2(const float* data) : m{{data[0], data[1]}, {data[2], data[3]}} {}

    ~Matrix2x2() = default;

    // 赋值
    Matrix2x2& operator=(const Matrix2x2& other);
    Matrix2x2& operator=(Matrix2x2&& other) noexcept;

    // 用[]访问行
    constexpr float* operator[](int index) { return m[index]; }
    constexpr const float* operator[](int index) const { return m[index]; }

    // 加减乘除
    constexpr Matrix2x2 operator+(const Matrix2x2& other) const {
        return Matrix2x2(
            m[0][0] + other.m[0][0], m[0][1] + other.m[0][1],
            m[1][0] + other.m[1][0], m[1][1] + other.m[1][1]
        );
    }
    constexpr Matrix2x2 operator-(const Matrix2x2& other) const {
        return Matrix2x2(
            m[0][0] - other.m[0][0], m[0][1] - other.m[0][1],
            m[1][0] - other.m[1][0], m[1][1] - other.m[1][1]
        );
    }
    // 矩阵乘法，这个要注意顺序
    constexpr Matrix2x2 operator*(const Matrix2x2& other) const {
        return Matrix2x2(
            m[0][0] * other.m[0][0] + m[0][1] * other.m[1][0],
            m[0][0] * other.m[0][1] + m[0][1] * other.m[1][1],
            m[1][0] * other.m[0][0] + m[1][1] * other.m[1][0],
            m[1][0] * other.m[0][1] + m[1][1] * other.m[1][1]
        );
    }
    constexpr Matrix2x2 operator*(float scalar) const {
        return Matrix2x2(
            m[0][0] * scalar, m[0][1] * scalar,
            m[1][0] * scalar, m[1][1] * scalar
        );
    }
    constexpr Matrix2x2 operator/(float scalar) const {
        float invScalar = 1.0f / scalar;
        return Matrix2x2(
            m[0][0] * invScalar, m[0][1] * invScalar,
            m[1][0] * invScalar, m[1][1] * invScalar
        );
    }

    // 复合赋值
    Matrix2x2& operator+=(const Matrix2x2& other);
    Matrix2x2& operator-=(const Matrix2x2& other);
    Matrix2x2& operator*=(const Matrix2x2& other);
    Matrix2x2& operator*=(float scalar);
    Matrix2x2& operator/=(float scalar);

    // 矩阵乘向量，做变换用
    constexpr Vector2 operator*(const Vector2& vector) const {
        return Vector2(
            m[0][0] * vector.x + m[0][1] * vector.y,
            m[1][0] * vector.x + m[1][1] * vector.y
        );
    }

    // 比较，浮点数要注意精度
    constexpr bool operator==(const Matrix2x2& other) const {
        return Math::abs(m[0][0] - other.m[0][0]) < Math::EPSILON &&
               Math::abs(m[0][1] - other.m[0][1]) < Math::EPSILON &&
               Math::abs(m[1][0] - other.m[1][0]) < Math::EPSILON &&
               Math::abs(m[1][1] - other.m[1][1]) < Math::EPSILON;
    }
    constexpr bool operator!=(const Matrix2x2& other) const { return !(*this == other); }

    // 转置，行列互换
    constexpr Matrix2x2 transposed() const {
        return Matrix2x2(m[0][0], m[1][0], m[0][1], m[1][1]);
    }
    Matrix2x2& transpose();
    // 行列式，2x2很简单
    constexpr float determinant() const { return m[0][0] * m[1][1] - m[0][1] * m[1][0]; }
    // 求逆，行列式为0就不行了
    Matrix2x2 inverse() const;
    Matrix2x2& invert();
    // 迹，对角线之和
    constexpr float trace() const { return m[0][0] + m[1][1]; }
    // 是不是单位矩阵
    constexpr bool isIdentity() const {
        return Math::abs(m[0][0] - 1.0f) < Math::EPSILON &&
               Math::abs(m[0][1]) < Math::EPSILON &&
               Math::abs(m[1][0]) < Math::EPSILON &&
               Math::abs(m[1][1] - 1.0f) < Math::EPSILON;
    }
    // 正交矩阵，转置等于逆
    constexpr bool isOrthogonal() const {
        Matrix2x2 t = transposed();
        Matrix2x2 product = *this * t;
        return product.isIdentity();
    }

    // 常量，直接用
    static const Matrix2x2 identity;
    static const Matrix2x2 zero;

    // 静态方法
    static constexpr Matrix2x2 Identity() { return Matrix2x2(1.0f, 0.0f, 0.0f, 1.0f); }
    static constexpr Matrix2x2 Zero() { return Matrix2x2(0.0f, 0.0f, 0.0f, 0.0f); }
    // 旋转矩阵，angle是弧度，除非isDegrees=true
    static Matrix2x2 rotation(float angle, bool isDegrees = false);
    static constexpr Matrix2x2 scaling(const Vector2& scale) {
        return Matrix2x2(scale.x, 0.0f, 0.0f, scale.y);
    }
    static constexpr Matrix2x2 scaling(float scale) {
        return Matrix2x2(scale, 0.0f, 0.0f, scale);
    }
    static constexpr Matrix2x2 fromVectors(const Vector2& v0, const Vector2& v1) {
        return Matrix2x2(v0, v1);
    }

    // 输出，调试用
    friend std::ostream& operator<<(std::ostream& os, const Matrix2x2& mat);
};

// 全局操作符，让 scalar * matrix 也能用
constexpr Matrix2x2 operator*(float scalar, const Matrix2x2& matrix) { return matrix * scalar; }
constexpr Matrix2x2 operator+(float scalar, const Matrix2x2& matrix) {
    return Matrix2x2(
        scalar + matrix.m[0][0], scalar + matrix.m[0][1],
        scalar + matrix.m[1][0], scalar + matrix.m[1][1]
    );
}
constexpr Matrix2x2 operator+(const Matrix2x2& matrix, float scalar) {
    return Matrix2x2(
        matrix.m[0][0] + scalar, matrix.m[0][1] + scalar,
        matrix.m[1][0] + scalar, matrix.m[1][1] + scalar
    );
}
constexpr Matrix2x2 operator-(const Matrix2x2& matrix, float scalar) {
    return Matrix2x2(
        matrix.m[0][0] - scalar, matrix.m[0][1] - scalar,
        matrix.m[1][0] - scalar, matrix.m[1][1] - scalar
    );
}
constexpr Matrix2x2 operator/(float scalar, const Matrix2x2& matrix) {
    return Matrix2x2(
        scalar / matrix.m[0][0], scalar / matrix.m[0][1],
        scalar / matrix.m[1][0], scalar / matrix.m[1][1]
    );
}

} // namespace AstraAlgebra
