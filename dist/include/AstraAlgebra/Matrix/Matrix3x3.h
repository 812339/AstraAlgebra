#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <iostream>

namespace AstraAlgebra {

// 3x3矩阵，3D旋转、缩放这些变换都用它
class Matrix3x3 {
public:
    // 3x3的数组，行优先存储
    float m[3][3];

    // 构造函数
    constexpr Matrix3x3() : m{{0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 0.0f}} {}
    constexpr Matrix3x3(float m00, float m01, float m02, float m10, float m11, float m12, float m20, float m21, float m22)
        : m{{m00, m01, m02}, {m10, m11, m12}, {m20, m21, m22}} {}
    // 标量构造，对角线是这个值
    constexpr Matrix3x3(float scalar) : m{{scalar, 0.0f, 0.0f}, {0.0f, scalar, 0.0f}, {0.0f, 0.0f, scalar}} {}
    constexpr Matrix3x3(const Matrix3x3& other)
        : m{{other.m[0][0], other.m[0][1], other.m[0][2]},
            {other.m[1][0], other.m[1][1], other.m[1][2]},
            {other.m[2][0], other.m[2][1], other.m[2][2]}} {}
    Matrix3x3(Matrix3x3&& other) noexcept {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                m[i][j] = other.m[i][j];
                other.m[i][j] = 0.0f;
            }
    }
    // 三个向量构造，每个向量是一行
    constexpr Matrix3x3(const Vector3& v0, const Vector3& v1, const Vector3& v2)
        : m{{v0.x, v0.y, v0.z}, {v1.x, v1.y, v1.z}, {v2.x, v2.y, v2.z}} {}
    constexpr Matrix3x3(const float* data)
        : m{{data[0], data[1], data[2]}, {data[3], data[4], data[5]}, {data[6], data[7], data[8]}} {}

    ~Matrix3x3() = default;

    // 赋值
    Matrix3x3& operator=(const Matrix3x3& other);
    Matrix3x3& operator=(Matrix3x3&& other) noexcept;

    // []访问行
    constexpr float* operator[](int index) { return m[index]; }
    constexpr const float* operator[](int index) const { return m[index]; }

    // 加减
    constexpr Matrix3x3 operator+(const Matrix3x3& other) const {
        return Matrix3x3(
            m[0][0] + other.m[0][0], m[0][1] + other.m[0][1], m[0][2] + other.m[0][2],
            m[1][0] + other.m[1][0], m[1][1] + other.m[1][1], m[1][2] + other.m[1][2],
            m[2][0] + other.m[2][0], m[2][1] + other.m[2][1], m[2][2] + other.m[2][2]
        );
    }
    constexpr Matrix3x3 operator-(const Matrix3x3& other) const {
        return Matrix3x3(
            m[0][0] - other.m[0][0], m[0][1] - other.m[0][1], m[0][2] - other.m[0][2],
            m[1][0] - other.m[1][0], m[1][1] - other.m[1][1], m[1][2] - other.m[1][2],
            m[2][0] - other.m[2][0], m[2][1] - other.m[2][1], m[2][2] - other.m[2][2]
        );
    }
    // 矩阵乘法，9个元素算起来有点繁琐
    constexpr Matrix3x3 operator*(const Matrix3x3& other) const {
        return Matrix3x3(
            m[0][0]*other.m[0][0] + m[0][1]*other.m[1][0] + m[0][2]*other.m[2][0],
            m[0][0]*other.m[0][1] + m[0][1]*other.m[1][1] + m[0][2]*other.m[2][1],
            m[0][0]*other.m[0][2] + m[0][1]*other.m[1][2] + m[0][2]*other.m[2][2],
            m[1][0]*other.m[0][0] + m[1][1]*other.m[1][0] + m[1][2]*other.m[2][0],
            m[1][0]*other.m[0][1] + m[1][1]*other.m[1][1] + m[1][2]*other.m[2][1],
            m[1][0]*other.m[0][2] + m[1][1]*other.m[1][2] + m[1][2]*other.m[2][2],
            m[2][0]*other.m[0][0] + m[2][1]*other.m[1][0] + m[2][2]*other.m[2][0],
            m[2][0]*other.m[0][1] + m[2][1]*other.m[1][1] + m[2][2]*other.m[2][1],
            m[2][0]*other.m[0][2] + m[2][1]*other.m[1][2] + m[2][2]*other.m[2][2]
        );
    }
    constexpr Matrix3x3 operator*(float scalar) const {
        return Matrix3x3(
            m[0][0]*scalar, m[0][1]*scalar, m[0][2]*scalar,
            m[1][0]*scalar, m[1][1]*scalar, m[1][2]*scalar,
            m[2][0]*scalar, m[2][1]*scalar, m[2][2]*scalar
        );
    }
    constexpr Matrix3x3 operator/(float scalar) const {
        float inv = 1.0f / scalar;
        return Matrix3x3(
            m[0][0]*inv, m[0][1]*inv, m[0][2]*inv,
            m[1][0]*inv, m[1][1]*inv, m[1][2]*inv,
            m[2][0]*inv, m[2][1]*inv, m[2][2]*inv
        );
    }

    // 复合赋值
    Matrix3x3& operator+=(const Matrix3x3& other);
    Matrix3x3& operator-=(const Matrix3x3& other);
    Matrix3x3& operator*=(const Matrix3x3& other);
    Matrix3x3& operator*=(float scalar);
    Matrix3x3& operator/=(float scalar);

    // 矩阵乘向量
    constexpr Vector3 operator*(const Vector3& vector) const {
        return Vector3(
            m[0][0]*vector.x + m[0][1]*vector.y + m[0][2]*vector.z,
            m[1][0]*vector.x + m[1][1]*vector.y + m[1][2]*vector.z,
            m[2][0]*vector.x + m[2][1]*vector.y + m[2][2]*vector.z
        );
    }

    // 比较，浮点数不能直接==
    constexpr bool operator==(const Matrix3x3& other) const {
        return Math::abs(m[0][0]-other.m[0][0])<Math::EPSILON && Math::abs(m[0][1]-other.m[0][1])<Math::EPSILON && Math::abs(m[0][2]-other.m[0][2])<Math::EPSILON &&
               Math::abs(m[1][0]-other.m[1][0])<Math::EPSILON && Math::abs(m[1][1]-other.m[1][1])<Math::EPSILON && Math::abs(m[1][2]-other.m[1][2])<Math::EPSILON &&
               Math::abs(m[2][0]-other.m[2][0])<Math::EPSILON && Math::abs(m[2][1]-other.m[2][1])<Math::EPSILON && Math::abs(m[2][2]-other.m[2][2])<Math::EPSILON;
    }
    constexpr bool operator!=(const Matrix3x3& other) const { return !(*this == other); }

    // 转置
    constexpr Matrix3x3 transposed() const {
        return Matrix3x3(
            m[0][0], m[1][0], m[2][0],
            m[0][1], m[1][1], m[2][1],
            m[0][2], m[1][2], m[2][2]
        );
    }
    Matrix3x3& transpose();
    // 行列式，3x3的公式有点长
    constexpr float determinant() const {
        return m[0][0]*(m[1][1]*m[2][2]-m[1][2]*m[2][1]) -
               m[0][1]*(m[1][0]*m[2][2]-m[1][2]*m[2][0]) +
               m[0][2]*(m[1][0]*m[2][1]-m[1][1]*m[2][0]);
    }
    // 逆矩阵
    Matrix3x3 inverse() const;
    Matrix3x3& invert();
    // 迹
    constexpr float trace() const { return m[0][0]+m[1][1]+m[2][2]; }
    // 是否单位矩阵
    constexpr bool isIdentity() const {
        return Math::abs(m[0][0]-1.0f)<Math::EPSILON && Math::abs(m[0][1])<Math::EPSILON && Math::abs(m[0][2])<Math::EPSILON &&
               Math::abs(m[1][0])<Math::EPSILON && Math::abs(m[1][1]-1.0f)<Math::EPSILON && Math::abs(m[1][2])<Math::EPSILON &&
               Math::abs(m[2][0])<Math::EPSILON && Math::abs(m[2][1])<Math::EPSILON && Math::abs(m[2][2]-1.0f)<Math::EPSILON;
    }
    // 正交矩阵
    constexpr bool isOrthogonal() const {
        Matrix3x3 t = transposed();
        Matrix3x3 product = *this * t;
        return product.isIdentity();
    }

    // 特征值和特征向量，这个计算量比较大
    float eigenvalues(float* eigenvalues) const;
    float eigenvectors(float* eigenvalues, Vector3* eigenvectors) const;
    float powerMethod(Vector3& eigenvector, float tolerance = 1e-6, int maxIterations = 1000) const;

    // 常量
    static const Matrix3x3 identity;
    static const Matrix3x3 zero;

    // 静态方法
    static constexpr Matrix3x3 Identity() { return Matrix3x3(1.0f,0.0f,0.0f, 0.0f,1.0f,0.0f, 0.0f,0.0f,1.0f); }
    static constexpr Matrix3x3 Zero() { return Matrix3x3(0.0f); }
    // 绕各轴旋转
    static Matrix3x3 rotationX(float angle, bool isDegrees = false);
    static Matrix3x3 rotationY(float angle, bool isDegrees = false);
    static Matrix3x3 rotationZ(float angle, bool isDegrees = false);
    // 绕任意轴旋转
    static Matrix3x3 rotation(const Vector3& axis, float angle, bool isDegrees = false);
    static constexpr Matrix3x3 scaling(const Vector3& scale) {
        return Matrix3x3(scale.x,0.0f,0.0f, 0.0f,scale.y,0.0f, 0.0f,0.0f,scale.z);
    }
    static constexpr Matrix3x3 scaling(float scale) {
        return Matrix3x3(scale,0.0f,0.0f, 0.0f,scale,0.0f, 0.0f,0.0f,scale);
    }
    static constexpr Matrix3x3 fromVectors(const Vector3& v0, const Vector3& v1, const Vector3& v2) {
        return Matrix3x3(v0, v1, v2);
    }

    // 输出
    friend std::ostream& operator<<(std::ostream& os, const Matrix3x3& mat);
};

// 全局操作符
constexpr Matrix3x3 operator*(float scalar, const Matrix3x3& matrix) { return matrix * scalar; }
constexpr Matrix3x3 operator+(float scalar, const Matrix3x3& matrix) {
    return Matrix3x3(
        scalar+matrix.m[0][0], scalar+matrix.m[0][1], scalar+matrix.m[0][2],
        scalar+matrix.m[1][0], scalar+matrix.m[1][1], scalar+matrix.m[1][2],
        scalar+matrix.m[2][0], scalar+matrix.m[2][1], scalar+matrix.m[2][2]
    );
}
constexpr Matrix3x3 operator+(const Matrix3x3& matrix, float scalar) {
    return Matrix3x3(
        matrix.m[0][0]+scalar, matrix.m[0][1]+scalar, matrix.m[0][2]+scalar,
        matrix.m[1][0]+scalar, matrix.m[1][1]+scalar, matrix.m[1][2]+scalar,
        matrix.m[2][0]+scalar, matrix.m[2][1]+scalar, matrix.m[2][2]+scalar
    );
}
constexpr Matrix3x3 operator-(const Matrix3x3& matrix, float scalar) {
    return Matrix3x3(
        matrix.m[0][0]-scalar, matrix.m[0][1]-scalar, matrix.m[0][2]-scalar,
        matrix.m[1][0]-scalar, matrix.m[1][1]-scalar, matrix.m[1][2]-scalar,
        matrix.m[2][0]-scalar, matrix.m[2][1]-scalar, matrix.m[2][2]-scalar
    );
}
constexpr Matrix3x3 operator/(float scalar, const Matrix3x3& matrix) {
    return Matrix3x3(
        scalar/matrix.m[0][0], scalar/matrix.m[0][1], scalar/matrix.m[0][2],
        scalar/matrix.m[1][0], scalar/matrix.m[1][1], scalar/matrix.m[1][2],
        scalar/matrix.m[2][0], scalar/matrix.m[2][1], scalar/matrix.m[2][2]
    );
}

} // namespace AstraAlgebra
