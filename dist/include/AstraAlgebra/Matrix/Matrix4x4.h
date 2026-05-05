// Matrix4x4 - 4x4矩阵，3D变换的核心，平移旋转缩放投影都靠它

#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <iostream>

namespace AstraAlgebra {

// 4x4矩阵，3D变换的核心，平移旋转缩放投影都靠它
class Matrix4x4 {
public:
    // 4x4数组，行优先
    float m[4][4];

    // 构造函数，默认是单位矩阵
    constexpr Matrix4x4() : m{{1.0f, 0.0f, 0.0f, 0.0f}, {0.0f, 1.0f, 0.0f, 0.0f}, {0.0f, 0.0f, 1.0f, 0.0f}, {0.0f, 0.0f, 0.0f, 1.0f}} {}
    constexpr Matrix4x4(float m00, float m01, float m02, float m03,
              float m10, float m11, float m12, float m13,
              float m20, float m21, float m22, float m23,
              float m30, float m31, float m32, float m33)
        : m{{m00, m01, m02, m03}, {m10, m11, m12, m13}, {m20, m21, m22, m23}, {m30, m31, m32, m33}} {}
    // 标量构造，对角线
    constexpr Matrix4x4(float scalar) : m{{scalar, 0.0f, 0.0f, 0.0f}, {0.0f, scalar, 0.0f, 0.0f}, {0.0f, 0.0f, scalar, 0.0f}, {0.0f, 0.0f, 0.0f, scalar}} {}
    constexpr Matrix4x4(const Matrix4x4& other) : m{{other.m[0][0], other.m[0][1], other.m[0][2], other.m[0][3]}, {other.m[1][0], other.m[1][1], other.m[1][2], other.m[1][3]}, {other.m[2][0], other.m[2][1], other.m[2][2], other.m[2][3]}, {other.m[3][0], other.m[3][1], other.m[3][2], other.m[3][3]}} {}
    constexpr Matrix4x4(Matrix4x4&& other) noexcept : m{{other.m[0][0], other.m[0][1], other.m[0][2], other.m[0][3]}, {other.m[1][0], other.m[1][1], other.m[1][2], other.m[1][3]}, {other.m[2][0], other.m[2][1], other.m[2][2], other.m[2][3]}, {other.m[3][0], other.m[3][1], other.m[3][2], other.m[3][3]}} {
        for (int i = 0; i < 4; ++i) for (int j = 0; j < 4; ++j) other.m[i][j] = 0.0f;
    }
    // 四个向量构造
    constexpr Matrix4x4(const Vector4& v0, const Vector4& v1, const Vector4& v2, const Vector4& v3)
        : m{{v0.x, v0.y, v0.z, v0.w}, {v1.x, v1.y, v1.z, v1.w}, {v2.x, v2.y, v2.z, v2.w}, {v3.x, v3.y, v3.z, v3.w}} {}
    constexpr Matrix4x4(const float* data) : m{{data[0], data[1], data[2], data[3]}, {data[4], data[5], data[6], data[7]}, {data[8], data[9], data[10], data[11]}, {data[12], data[13], data[14], data[15]}} {}

    ~Matrix4x4() = default;

    // 赋值
    Matrix4x4& operator=(const Matrix4x4& other);
    Matrix4x4& operator=(Matrix4x4&& other) noexcept;

    // []访问行
    constexpr float* operator[](int index) { return m[index]; }
    [[nodiscard]] constexpr const float* operator[](int index) const { return m[index]; }

    // 加减
    [[nodiscard]] constexpr Matrix4x4 operator+(const Matrix4x4& other) const {
        return Matrix4x4(
            m[0][0]+other.m[0][0], m[0][1]+other.m[0][1], m[0][2]+other.m[0][2], m[0][3]+other.m[0][3],
            m[1][0]+other.m[1][0], m[1][1]+other.m[1][1], m[1][2]+other.m[1][2], m[1][3]+other.m[1][3],
            m[2][0]+other.m[2][0], m[2][1]+other.m[2][1], m[2][2]+other.m[2][2], m[2][3]+other.m[2][3],
            m[3][0]+other.m[3][0], m[3][1]+other.m[3][1], m[3][2]+other.m[3][2], m[3][3]+other.m[3][3]
        );
    }
    [[nodiscard]] constexpr Matrix4x4 operator-(const Matrix4x4& other) const {
        return Matrix4x4(
            m[0][0]-other.m[0][0], m[0][1]-other.m[0][1], m[0][2]-other.m[0][2], m[0][3]-other.m[0][3],
            m[1][0]-other.m[1][0], m[1][1]-other.m[1][1], m[1][2]-other.m[1][2], m[1][3]-other.m[1][3],
            m[2][0]-other.m[2][0], m[2][1]-other.m[2][1], m[2][2]-other.m[2][2], m[2][3]-other.m[2][3],
            m[3][0]-other.m[3][0], m[3][1]-other.m[3][1], m[3][2]-other.m[3][2], m[3][3]-other.m[3][3]
        );
    }
    // 矩阵乘法，16个元素，用局部变量优化一下减少内存访问
    [[nodiscard]] constexpr Matrix4x4 operator*(const Matrix4x4& other) const {
        float a00 = m[0][0], a01 = m[0][1], a02 = m[0][2], a03 = m[0][3];
        float a10 = m[1][0], a11 = m[1][1], a12 = m[1][2], a13 = m[1][3];
        float a20 = m[2][0], a21 = m[2][1], a22 = m[2][2], a23 = m[2][3];
        float a30 = m[3][0], a31 = m[3][1], a32 = m[3][2], a33 = m[3][3];
        
        float b00 = other.m[0][0], b01 = other.m[0][1], b02 = other.m[0][2], b03 = other.m[0][3];
        float b10 = other.m[1][0], b11 = other.m[1][1], b12 = other.m[1][2], b13 = other.m[1][3];
        float b20 = other.m[2][0], b21 = other.m[2][1], b22 = other.m[2][2], b23 = other.m[2][3];
        float b30 = other.m[3][0], b31 = other.m[3][1], b32 = other.m[3][2], b33 = other.m[3][3];
        
        return Matrix4x4(
            a00*b00 + a01*b10 + a02*b20 + a03*b30,
            a00*b01 + a01*b11 + a02*b21 + a03*b31,
            a00*b02 + a01*b12 + a02*b22 + a03*b32,
            a00*b03 + a01*b13 + a02*b23 + a03*b33,
            a10*b00 + a11*b10 + a12*b20 + a13*b30,
            a10*b01 + a11*b11 + a12*b21 + a13*b31,
            a10*b02 + a11*b12 + a12*b22 + a13*b32,
            a10*b03 + a11*b13 + a12*b23 + a13*b33,
            a20*b00 + a21*b10 + a22*b20 + a23*b30,
            a20*b01 + a21*b11 + a22*b21 + a23*b31,
            a20*b02 + a21*b12 + a22*b22 + a23*b32,
            a20*b03 + a21*b13 + a22*b23 + a23*b33,
            a30*b00 + a31*b10 + a32*b20 + a33*b30,
            a30*b01 + a31*b11 + a32*b21 + a33*b31,
            a30*b02 + a31*b12 + a32*b22 + a33*b32,
            a30*b03 + a31*b13 + a32*b23 + a33*b33
        );
    }
    [[nodiscard]] constexpr Matrix4x4 operator*(float scalar) const {
        return Matrix4x4(
            m[0][0]*scalar, m[0][1]*scalar, m[0][2]*scalar, m[0][3]*scalar,
            m[1][0]*scalar, m[1][1]*scalar, m[1][2]*scalar, m[1][3]*scalar,
            m[2][0]*scalar, m[2][1]*scalar, m[2][2]*scalar, m[2][3]*scalar,
            m[3][0]*scalar, m[3][1]*scalar, m[3][2]*scalar, m[3][3]*scalar
        );
    }
    [[nodiscard]] constexpr Matrix4x4 operator/(float scalar) const {
        float inv = 1.0f / scalar;
        return Matrix4x4(
            m[0][0]*inv, m[0][1]*inv, m[0][2]*inv, m[0][3]*inv,
            m[1][0]*inv, m[1][1]*inv, m[1][2]*inv, m[1][3]*inv,
            m[2][0]*inv, m[2][1]*inv, m[2][2]*inv, m[2][3]*inv,
            m[3][0]*inv, m[3][1]*inv, m[3][2]*inv, m[3][3]*inv
        );
    }

    // 复合赋值
    Matrix4x4& operator+=(const Matrix4x4& other);
    Matrix4x4& operator-=(const Matrix4x4& other);
    Matrix4x4& operator*=(const Matrix4x4& other);
    Matrix4x4& operator*=(float scalar);
    Matrix4x4& operator/=(float scalar);

    // 矩阵乘向量，注意Vector4和Vector3版本不一样
    [[nodiscard]] constexpr Vector4 operator*(const Vector4& vector) const {
        return Vector4(
            m[0][0]*vector.x + m[0][1]*vector.y + m[0][2]*vector.z + m[0][3]*vector.w,
            m[1][0]*vector.x + m[1][1]*vector.y + m[1][2]*vector.z + m[1][3]*vector.w,
            m[2][0]*vector.x + m[2][1]*vector.y + m[2][2]*vector.z + m[2][3]*vector.w,
            m[3][0]*vector.x + m[3][1]*vector.y + m[3][2]*vector.z + m[3][3]*vector.w
        );
    }
    // Vector3版本会做透视除法，w是算出来的
    [[nodiscard]] constexpr Vector3 operator*(const Vector3& vector) const {
        float w = m[3][0]*vector.x + m[3][1]*vector.y + m[3][2]*vector.z + m[3][3];
        return Vector3(
            (m[0][0]*vector.x + m[0][1]*vector.y + m[0][2]*vector.z + m[0][3]) / w,
            (m[1][0]*vector.x + m[1][1]*vector.y + m[1][2]*vector.z + m[1][3]) / w,
            (m[2][0]*vector.x + m[2][1]*vector.y + m[2][2]*vector.z + m[2][3]) / w
        );
    }

    // 比较，16个元素都要比
    [[nodiscard]] constexpr bool operator==(const Matrix4x4& other) const {
        return Math::abs(m[0][0]-other.m[0][0])<Math::EPSILON && Math::abs(m[0][1]-other.m[0][1])<Math::EPSILON &&
               Math::abs(m[0][2]-other.m[0][2])<Math::EPSILON && Math::abs(m[0][3]-other.m[0][3])<Math::EPSILON &&
               Math::abs(m[1][0]-other.m[1][0])<Math::EPSILON && Math::abs(m[1][1]-other.m[1][1])<Math::EPSILON &&
               Math::abs(m[1][2]-other.m[1][2])<Math::EPSILON && Math::abs(m[1][3]-other.m[1][3])<Math::EPSILON &&
               Math::abs(m[2][0]-other.m[2][0])<Math::EPSILON && Math::abs(m[2][1]-other.m[2][1])<Math::EPSILON &&
               Math::abs(m[2][2]-other.m[2][2])<Math::EPSILON && Math::abs(m[2][3]-other.m[2][3])<Math::EPSILON &&
               Math::abs(m[3][0]-other.m[3][0])<Math::EPSILON && Math::abs(m[3][1]-other.m[3][1])<Math::EPSILON &&
               Math::abs(m[3][2]-other.m[3][2])<Math::EPSILON && Math::abs(m[3][3]-other.m[3][3])<Math::EPSILON;
    }
    [[nodiscard]] constexpr bool operator!=(const Matrix4x4& other) const { return !(*this == other); }

    // 转置
    [[nodiscard]] constexpr Matrix4x4 transposed() const {
        return Matrix4x4(
            m[0][0], m[1][0], m[2][0], m[3][0],
            m[0][1], m[1][1], m[2][1], m[3][1],
            m[0][2], m[1][2], m[2][2], m[3][2],
            m[0][3], m[1][3], m[2][3], m[3][3]
        );
    }
    Matrix4x4& transposeInPlace();
    // 行列式，4x4的有点复杂
    [[nodiscard]] float determinant() const;
    // 逆矩阵
    [[nodiscard]] Matrix4x4 inverse() const;
    Matrix4x4& invert();
    // 迹
    [[nodiscard]] constexpr float trace() const { return m[0][0]+m[1][1]+m[2][2]+m[3][3]; }
    // 是否单位矩阵
    [[nodiscard]] constexpr bool isIdentity() const {
        return Math::abs(m[0][0]-1.0f)<Math::EPSILON && Math::abs(m[0][1])<Math::EPSILON && Math::abs(m[0][2])<Math::EPSILON && Math::abs(m[0][3])<Math::EPSILON &&
               Math::abs(m[1][0])<Math::EPSILON && Math::abs(m[1][1]-1.0f)<Math::EPSILON && Math::abs(m[1][2])<Math::EPSILON && Math::abs(m[1][3])<Math::EPSILON &&
               Math::abs(m[2][0])<Math::EPSILON && Math::abs(m[2][1])<Math::EPSILON && Math::abs(m[2][2]-1.0f)<Math::EPSILON && Math::abs(m[2][3])<Math::EPSILON &&
               Math::abs(m[3][0])<Math::EPSILON && Math::abs(m[3][1])<Math::EPSILON && Math::abs(m[3][2])<Math::EPSILON && Math::abs(m[3][3]-1.0f)<Math::EPSILON;
    }
    // 正交矩阵
    [[nodiscard]] constexpr bool isOrthogonal() const {
        Matrix4x4 t = transposed();
        Matrix4x4 product = *this * t;
        return product.isIdentity();
    }

    // 变换相关方法
    // 变换点，会做透视除法
    [[nodiscard]] constexpr Vector3 transformPoint(const Vector3& point) const {
        float w = m[3][0]*point.x + m[3][1]*point.y + m[3][2]*point.z + m[3][3];
        return Vector3(
            (m[0][0]*point.x + m[0][1]*point.y + m[0][2]*point.z + m[0][3]) / w,
            (m[1][0]*point.x + m[1][1]*point.y + m[1][2]*point.z + m[1][3]) / w,
            (m[2][0]*point.x + m[2][1]*point.y + m[2][2]*point.z + m[2][3]) / w
        );
    }
    // 变换向量，不考虑平移部分
    [[nodiscard]] constexpr Vector3 transformVector(const Vector3& vector) const {
        return Vector3(
            m[0][0]*vector.x + m[0][1]*vector.y + m[0][2]*vector.z,
            m[1][0]*vector.x + m[1][1]*vector.y + m[1][2]*vector.z,
            m[2][0]*vector.x + m[2][1]*vector.y + m[2][2]*vector.z
        );
    }
    // 变换方向，跟transformVector一样，语义不同而已
    [[nodiscard]] constexpr Vector3 transformDirection(const Vector3& direction) const {
        return Vector3(
            m[0][0]*direction.x + m[0][1]*direction.y + m[0][2]*direction.z,
            m[1][0]*direction.x + m[1][1]*direction.y + m[1][2]*direction.z,
            m[2][0]*direction.x + m[2][1]*direction.y + m[2][2]*direction.z
        );
    }

    // 分解变换
    [[nodiscard]] constexpr Vector3 getTranslation() const { return Vector3(m[0][3], m[1][3], m[2][3]); }
    void setTranslation(const Vector3& translation);
    [[nodiscard]] Matrix3x3 getRotationMatrix() const;
    [[nodiscard]] Vector3 getScale() const;
    // 分解成平移、旋转、缩放
    void decompose(Vector3& position, Quaternion& rotation, Vector3& scale) const;

    // 常量
    static const Matrix4x4 identity;
    static const Matrix4x4 zero;

    // 静态方法
    static constexpr Matrix4x4 Identity() { return Matrix4x4(1.0f,0.0f,0.0f,0.0f, 0.0f,1.0f,0.0f,0.0f, 0.0f,0.0f,1.0f,0.0f, 0.0f,0.0f,0.0f,1.0f); }
    static constexpr Matrix4x4 Zero() { return Matrix4x4(0.0f); }
    // 平移矩阵
    static constexpr Matrix4x4 translation(const Vector3& t) {
        return Matrix4x4(1.0f,0.0f,0.0f,t.x, 0.0f,1.0f,0.0f,t.y, 0.0f,0.0f,1.0f,t.z, 0.0f,0.0f,0.0f,1.0f);
    }
    static constexpr Matrix4x4 scale(const Vector3& s) { return scaling(s); }
    // 缩放矩阵
    static constexpr Matrix4x4 scaling(const Vector3& s) {
        return Matrix4x4(s.x,0.0f,0.0f,0.0f, 0.0f,s.y,0.0f,0.0f, 0.0f,0.0f,s.z,0.0f, 0.0f,0.0f,0.0f,1.0f);
    }
    static constexpr Matrix4x4 scale(float s) { return scaling(s); }
    static constexpr Matrix4x4 scaling(float s) {
        return Matrix4x4(s,0.0f,0.0f,0.0f, 0.0f,s,0.0f,0.0f, 0.0f,0.0f,s,0.0f, 0.0f,0.0f,0.0f,1.0f);
    }
    // 旋转
    static Matrix4x4 rotationX(float angle, bool isDegrees = false);
    static Matrix4x4 rotationY(float angle, bool isDegrees = false);
    static Matrix4x4 rotationZ(float angle, bool isDegrees = false);
    static Matrix4x4 rotation(const Vector3& axis, float angle, bool isDegrees = false);
    static Matrix4x4 rotation(const Vector3& eulerAngles, bool isDegrees = false);
    // 相机相关
    static Matrix4x4 lookAt(const Vector3& eye, const Vector3& target, const Vector3& up);
    // 透视投影
    static Matrix4x4 perspective(float fov, float aspectRatio, float nearPlane, float farPlane);
    // 正交投影
    static Matrix4x4 orthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane);
    static constexpr Matrix4x4 fromVectors(const Vector4& v0, const Vector4& v1, const Vector4& v2, const Vector4& v3) {
        return Matrix4x4(v0, v1, v2, v3);
    }
    static Matrix4x4 fromRotationMatrix(const Matrix3x3& rotation);
    // 从TRS构造矩阵
    static Matrix4x4 fromTRS(const Vector3& translation, const Quaternion& rotation, const Vector3& scale);
    static Matrix4x4 trs(const Vector3& translation, const Quaternion& rotation, const Vector3& scale) {
        return fromTRS(translation, rotation, scale);
    }
    // 其他投影方法
    static Matrix4x4 perspectiveFov(float fovY, float aspect, float zNear, float zFar, bool isDegrees = false);
    static Matrix4x4 perspectiveOffCenter(float left, float right, float bottom, float top, float nearPlane, float farPlane);
    static Matrix4x4 ortho(float left, float right, float bottom, float top, float nearPlane = -1.0f, float farPlane = 1.0f);
    static Matrix4x4 lookTo(const Vector3& eye, const Vector3& forward, const Vector3& up);
    // 反射和阴影矩阵
    static Matrix4x4 reflection(const Vector3& point, const Vector3& normal);
    static Matrix4x4 shadow(const Vector3& lightPos, const Vector3& planeNormal, float planeD);
    // 坐标系转换
    static Matrix4x4 worldToLocal(const Vector3& position, const Quaternion& rotation, const Vector3& scale);
    static Matrix4x4 localToWorld(const Vector3& position, const Quaternion& rotation, const Vector3& scale);
    // 提取旋转部分，去掉缩放
    constexpr Matrix4x4 extractRotation() const {
        Vector3 sx(m[0][0], m[1][0], m[2][0]);
        Vector3 sy(m[0][1], m[1][1], m[2][1]);
        Vector3 sz(m[0][2], m[1][2], m[2][2]);
        float sxLen = sx.length();
        float syLen = sy.length();
        float szLen = sz.length();
        if (sxLen < Math::EPSILON || syLen < Math::EPSILON || szLen < Math::EPSILON) return Matrix4x4::identity;
        return Matrix4x4(
            sx.x/sxLen, sy.x/syLen, sz.x/szLen, 0.0f,
            sx.y/sxLen, sy.y/syLen, sz.y/szLen, 0.0f,
            sx.z/sxLen, sy.z/syLen, sz.z/szLen, 0.0f,
            0.0f, 0.0f, 0.0f, 1.0f
        );
    }
    // 别名方法
    float getDeterminant() const { return determinant(); }
    Matrix4x4 getInverse() const { return inverse(); }
    constexpr Matrix4x4 getTranspose() const { return transposed(); }

    // 输出
    friend std::ostream& operator<<(std::ostream& os, const Matrix4x4& mat);
};

// 全局操作符
constexpr Matrix4x4 operator*(float scalar, const Matrix4x4& matrix) { return matrix * scalar; }
constexpr Matrix4x4 operator+(float scalar, const Matrix4x4& matrix) {
    return Matrix4x4(
        scalar+matrix.m[0][0], scalar+matrix.m[0][1], scalar+matrix.m[0][2], scalar+matrix.m[0][3],
        scalar+matrix.m[1][0], scalar+matrix.m[1][1], scalar+matrix.m[1][2], scalar+matrix.m[1][3],
        scalar+matrix.m[2][0], scalar+matrix.m[2][1], scalar+matrix.m[2][2], scalar+matrix.m[2][3],
        scalar+matrix.m[3][0], scalar+matrix.m[3][1], scalar+matrix.m[3][2], scalar+matrix.m[3][3]
    );
}
constexpr Matrix4x4 operator+(const Matrix4x4& matrix, float scalar) {
    return Matrix4x4(
        matrix.m[0][0]+scalar, matrix.m[0][1]+scalar, matrix.m[0][2]+scalar, matrix.m[0][3]+scalar,
        matrix.m[1][0]+scalar, matrix.m[1][1]+scalar, matrix.m[1][2]+scalar, matrix.m[1][3]+scalar,
        matrix.m[2][0]+scalar, matrix.m[2][1]+scalar, matrix.m[2][2]+scalar, matrix.m[2][3]+scalar,
        matrix.m[3][0]+scalar, matrix.m[3][1]+scalar, matrix.m[3][2]+scalar, matrix.m[3][3]+scalar
    );
}
constexpr Matrix4x4 operator-(const Matrix4x4& matrix, float scalar) {
    return Matrix4x4(
        matrix.m[0][0]-scalar, matrix.m[0][1]-scalar, matrix.m[0][2]-scalar, matrix.m[0][3]-scalar,
        matrix.m[1][0]-scalar, matrix.m[1][1]-scalar, matrix.m[1][2]-scalar, matrix.m[1][3]-scalar,
        matrix.m[2][0]-scalar, matrix.m[2][1]-scalar, matrix.m[2][2]-scalar, matrix.m[2][3]-scalar,
        matrix.m[3][0]-scalar, matrix.m[3][1]-scalar, matrix.m[3][2]-scalar, matrix.m[3][3]-scalar
    );
}
constexpr Matrix4x4 operator/(float scalar, const Matrix4x4& matrix) {
    return Matrix4x4(
        scalar/matrix.m[0][0], scalar/matrix.m[0][1], scalar/matrix.m[0][2], scalar/matrix.m[0][3],
        scalar/matrix.m[1][0], scalar/matrix.m[1][1], scalar/matrix.m[1][2], scalar/matrix.m[1][3],
        scalar/matrix.m[2][0], scalar/matrix.m[2][1], scalar/matrix.m[2][2], scalar/matrix.m[2][3],
        scalar/matrix.m[3][0], scalar/matrix.m[3][1], scalar/matrix.m[3][2], scalar/matrix.m[3][3]
    );
}

} // namespace AstraAlgebra
