// Matrix4x4d - 双精度4x4矩阵，高精度场景用的，一般游戏float就够了

#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Vector/Vector3d.h>
#include <iostream>

namespace AstraAlgebra {

class Matrix3x3d;

// 双精度4x4矩阵，高精度场景用的，一般游戏开发float就够了
class Matrix4x4d {
public:
    double m[4][4];

    // 构造函数，默认是单位矩阵
    constexpr Matrix4x4d() : m{{1.0, 0.0, 0.0, 0.0},
                               {0.0, 1.0, 0.0, 0.0},
                               {0.0, 0.0, 1.0, 0.0},
                               {0.0, 0.0, 0.0, 1.0}} {}
    constexpr Matrix4x4d(double m00, double m01, double m02, double m03,
                        double m10, double m11, double m12, double m13,
                        double m20, double m21, double m22, double m23,
                        double m30, double m31, double m32, double m33) 
        : m{{m00, m01, m02, m03},
            {m10, m11, m12, m13},
            {m20, m21, m22, m23},
            {m30, m31, m32, m33}} {}
    constexpr Matrix4x4d(const Matrix4x4d& other) 
        : m{{other.m[0][0], other.m[0][1], other.m[0][2], other.m[0][3]},
            {other.m[1][0], other.m[1][1], other.m[1][2], other.m[1][3]},
            {other.m[2][0], other.m[2][1], other.m[2][2], other.m[2][3]},
            {other.m[3][0], other.m[3][1], other.m[3][2], other.m[3][3]}} {}

    // 赋值
    Matrix4x4d& operator=(const Matrix4x4d& other) {
        if (this != &other) {
            for (int i = 0; i < 4; ++i)
                for (int j = 0; j < 4; ++j)
                    m[i][j] = other.m[i][j];
        }
        return *this;
    }

    // 用()访问元素，跟别的矩阵库不太一样
    constexpr double& operator()(int row, int col) { return m[row][col]; }
    constexpr const double& operator()(int row, int col) const { return m[row][col]; }

    // 加减
    constexpr Matrix4x4d operator+(const Matrix4x4d& other) const {
        Matrix4x4d result;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                result.m[i][j] = m[i][j] + other.m[i][j];
        return result;
    }
    constexpr Matrix4x4d operator-(const Matrix4x4d& other) const {
        Matrix4x4d result;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                result.m[i][j] = m[i][j] - other.m[i][j];
        return result;
    }
    constexpr Matrix4x4d operator*(double scalar) const {
        Matrix4x4d result;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                result.m[i][j] = m[i][j] * scalar;
        return result;
    }
    // 矩阵乘法，用循环写的，性能一般但双精度本来就不追求极致速度
    constexpr Matrix4x4d operator*(const Matrix4x4d& other) const {
        Matrix4x4d result;
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                result.m[i][j] = 0.0;
                for (int k = 0; k < 4; ++k) {
                    result.m[i][j] += m[i][k] * other.m[k][j];
                }
            }
        }
        return result;
    }

    // 变换向量，会做透视除法
    constexpr Vector3d operator*(const Vector3d& v) const {
        double w = m[3][0] * v.x + m[3][1] * v.y + m[3][2] * v.z + m[3][3];
        return Vector3d(
            (m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z + m[0][3]) / w,
            (m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z + m[1][3]) / w,
            (m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z + m[2][3]) / w
        );
    }

    // 复合赋值
    Matrix4x4d& operator+=(const Matrix4x4d& other) {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m[i][j] += other.m[i][j];
        return *this;
    }
    Matrix4x4d& operator*=(double scalar) {
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                m[i][j] *= scalar;
        return *this;
    }
    Matrix4x4d& operator*=(const Matrix4x4d& other) {
        *this = *this * other;
        return *this;
    }

    // 转置
    constexpr Matrix4x4d transposed() const {
        Matrix4x4d result;
        for (int i = 0; i < 4; ++i)
            for (int j = 0; j < 4; ++j)
                result.m[i][j] = m[j][i];
        return result;
    }
    Matrix4x4d& transpose() {
        *this = transposed();
        return *this;
    }

    // 行列式和逆矩阵，实现比较长，在cpp里
    double determinant() const;
    Matrix4x4d inverse() const;

    // 静态工厂方法
    static constexpr Matrix4x4d identityMatrix() {
        return Matrix4x4d(
            1.0, 0.0, 0.0, 0.0,
            0.0, 1.0, 0.0, 0.0,
            0.0, 0.0, 1.0, 0.0,
            0.0, 0.0, 0.0, 1.0
        );
    }

    static constexpr Matrix4x4d zeroMatrix() {
        return Matrix4x4d(
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0,
            0.0, 0.0, 0.0, 0.0
        );
    }

    // 平移
    static constexpr Matrix4x4d translation(const Vector3d& t) {
        Matrix4x4d result = identityMatrix();
        result.m[0][3] = t.x;
        result.m[1][3] = t.y;
        result.m[2][3] = t.z;
        return result;
    }

    // 缩放
    static constexpr Matrix4x4d scale(const Vector3d& s) {
        Matrix4x4d result = identityMatrix();
        result.m[0][0] = s.x;
        result.m[1][1] = s.y;
        result.m[2][2] = s.z;
        return result;
    }

    static constexpr Matrix4x4d scale(double s) {
        Matrix4x4d result = identityMatrix();
        result.m[0][0] = s;
        result.m[1][1] = s;
        result.m[2][2] = s;
        return result;
    }

    // 旋转
    static constexpr Matrix4x4d rotationX(double angle, bool isDegrees = false) {
        if (isDegrees) angle = Math::degToRad(angle);
        double c = Math::cos(angle);
        double s = Math::sin(angle);
        Matrix4x4d result = identityMatrix();
        result.m[1][1] = c; result.m[1][2] = -s;
        result.m[2][1] = s; result.m[2][2] = c;
        return result;
    }

    static constexpr Matrix4x4d rotationY(double angle, bool isDegrees = false) {
        if (isDegrees) angle = Math::degToRad(angle);
        double c = Math::cos(angle);
        double s = Math::sin(angle);
        Matrix4x4d result = identityMatrix();
        result.m[0][0] = c; result.m[0][2] = s;
        result.m[2][0] = -s; result.m[2][2] = c;
        return result;
    }

    static constexpr Matrix4x4d rotationZ(double angle, bool isDegrees = false) {
        if (isDegrees) angle = Math::degToRad(angle);
        double c = Math::cos(angle);
        double s = Math::sin(angle);
        Matrix4x4d result = identityMatrix();
        result.m[0][0] = c; result.m[0][1] = -s;
        result.m[1][0] = s; result.m[1][1] = c;
        return result;
    }

    // 相机和投影
    static Matrix4x4d lookAt(const Vector3d& eye, const Vector3d& target, const Vector3d& up = Vector3d::up);
    static Matrix4x4d perspective(double fov, double aspect, double near, double far);
    static Matrix4x4d orthographic(double left, double right, double bottom, double top, double near, double far);

    // 常量
    static const Matrix4x4d identity;
    static const Matrix4x4d zero;

    // 输出
    friend std::ostream& operator<<(std::ostream& os, const Matrix4x4d& mat) {
        for (int i = 0; i < 4; ++i) {
            os << "[";
            for (int j = 0; j < 4; ++j) {
                os << mat.m[i][j];
                if (j < 3) os << ", ";
            }
            os << "]";
            if (i < 3) os << std::endl;
        }
        return os;
    }
};

inline const Matrix4x4d Matrix4x4d::identity = Matrix4x4d::identityMatrix();
inline const Matrix4x4d Matrix4x4d::zero = Matrix4x4d::zeroMatrix();

// 全局操作符
constexpr Matrix4x4d operator*(double scalar, const Matrix4x4d& mat) { return mat * scalar; }

} // namespace AstraAlgebra
