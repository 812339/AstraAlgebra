// Quaternion - 四元数，3D旋转用的，比欧拉角好使，不会万向节死锁

#pragma once

#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <iostream>

// SIMD支持
#if defined(__SSE2__) || defined(__AVX__) || defined(__AVX2__) || defined(_M_X64) || (defined(_M_IX86_FP) && _M_IX86_FP >= 2)
#include <emmintrin.h>
#endif

namespace AstraAlgebra {

// 前向声明
class Matrix4x4;

// 四元数，表示3D旋转的，xyz是虚部，w是实部，别搞混了
class Quaternion {
public:
    // xyzw四个分量
    float x, y, z, w;

    // 构造函数，有好几种
    constexpr Quaternion() : x(0.0f), y(0.0f), z(0.0f), w(1.0f) {}
    constexpr Quaternion(float x, float y, float z, float w) : x(x), y(y), z(z), w(w) {}
    constexpr Quaternion(float scalar) : x(0.0f), y(0.0f), z(0.0f), w(scalar) {}
    constexpr Quaternion(const Quaternion& other) : x(other.x), y(other.y), z(other.z), w(other.w) {}
    constexpr Quaternion(Quaternion&& other) noexcept : x(other.x), y(other.y), z(other.z), w(other.w) {
        other.x = 0.0f; other.y = 0.0f; other.z = 0.0f; other.w = 0.0f;
    }
    // 用轴角构造
    Quaternion(const Vector3& axis, float angle, bool isDegrees = false);
    // 用欧拉角构造，注意顺序
    Quaternion(const Vector3& eulerAngles, bool isDegrees = false);
    // 用旋转矩阵构造
    Quaternion(const Matrix3x3& rotationMatrix);
    Quaternion(const Matrix4x4& rotationMatrix);

    // 析构函数，默认的就行
    ~Quaternion() = default;

    // 赋值
    Quaternion& operator=(const Quaternion& other);
    Quaternion& operator=(Quaternion&& other) noexcept;

    // 用[]访问分量
    constexpr float& operator[](int index) { return (&x)[index]; }
    constexpr const float& operator[](int index) const { return (&x)[index]; }

    // 加减乘除
    constexpr Quaternion operator+(const Quaternion& other) const {
        return Quaternion(x + other.x, y + other.y, z + other.z, w + other.w);
    }
    constexpr Quaternion operator-(const Quaternion& other) const {
        return Quaternion(x - other.x, y - other.y, z - other.z, w - other.w);
    }
    // 四元数乘法
    [[nodiscard]] constexpr Quaternion operator*(const Quaternion& other) const {
        return Quaternion(
            w * other.x + x * other.w + y * other.z - z * other.y,
            w * other.y - x * other.z + y * other.w + z * other.x,
            w * other.z + x * other.y - y * other.x + z * other.w,
            w * other.w - x * other.x - y * other.y - z * other.z
        );
    }
    constexpr Quaternion operator*(float scalar) const {
        return Quaternion(x * scalar, y * scalar, z * scalar, w * scalar);
    }
    constexpr Quaternion operator/(float scalar) const {
        float inv = 1.0f / scalar;
        return Quaternion(x * inv, y * inv, z * inv, w * inv);
    }
    // 四元数旋转向量，游戏里经常用
    // 使用 t = 2 * cross(q.xyz, v) 公式，比旋转矩阵法少一半运算
    [[nodiscard]] Vector3 operator*(const Vector3& vector) const {
#if defined(__SSE2__)
        // SSE向量化：并行计算所有分量
        __m128 q_vec = _mm_set_ps(w, z, y, x);
        __m128 v_vec = _mm_set_ps(0.0f, vector.z, vector.y, vector.x);
        
        // 提取q.xyz（将w分量置零）
        __m128 xyz_mask = _mm_castsi128_ps(_mm_set_epi32(0, -1, -1, -1));
        __m128 q_xyz = _mm_and_ps(q_vec, xyz_mask);
        
        // t = 2 * cross(q.xyz, v)
        __m128 q_yzx = _mm_shuffle_ps(q_xyz, q_xyz, _MM_SHUFFLE(3,0,2,1));
        __m128 v_zxy = _mm_shuffle_ps(v_vec, v_vec, _MM_SHUFFLE(3,1,0,2));
        __m128 q_zxy = _mm_shuffle_ps(q_xyz, q_xyz, _MM_SHUFFLE(3,1,0,2));
        __m128 v_yzx = _mm_shuffle_ps(v_vec, v_vec, _MM_SHUFFLE(3,0,2,1));
        
        __m128 cross = _mm_sub_ps(_mm_mul_ps(q_yzx, v_zxy), _mm_mul_ps(q_zxy, v_yzx));
        __m128 t = _mm_add_ps(cross, cross);
        
        // result = v + w*t + cross(q.xyz, t)
        __m128 w_vec = _mm_shuffle_ps(q_vec, q_vec, _MM_SHUFFLE(3,3,3,3));
        __m128 w_t = _mm_mul_ps(w_vec, t);
        
        __m128 t_zxy = _mm_shuffle_ps(t, t, _MM_SHUFFLE(3,1,0,2));
        __m128 t_yzx = _mm_shuffle_ps(t, t, _MM_SHUFFLE(3,0,2,1));
        __m128 cross2 = _mm_sub_ps(_mm_mul_ps(q_yzx, t_zxy), _mm_mul_ps(q_zxy, t_yzx));
        
        __m128 result = _mm_add_ps(v_vec, _mm_add_ps(w_t, cross2));
        
        Vector3 res;
        _mm_store_ss(&res.x, result);
        _mm_store_ss(&res.y, _mm_shuffle_ps(result, result, _MM_SHUFFLE(1,1,1,1)));
        _mm_store_ss(&res.z, _mm_shuffle_ps(result, result, _MM_SHUFFLE(2,2,2,2)));
        return res;
#else
        float t1 = 2.0f * (y * vector.z - z * vector.y);
        float t2 = 2.0f * (z * vector.x - x * vector.z);
        float t3 = 2.0f * (x * vector.y - y * vector.x);
        
        return Vector3(
            vector.x + w * t1 + (y * t3 - z * t2),
            vector.y + w * t2 + (z * t1 - x * t3),
            vector.z + w * t3 + (x * t2 - y * t1)
        );
#endif
    }

    // 复合赋值
    Quaternion& operator+=(const Quaternion& other);
    Quaternion& operator-=(const Quaternion& other);
    Quaternion& operator*=(const Quaternion& other);
    Quaternion& operator*=(float scalar);
    Quaternion& operator/=(float scalar);

    // 取反和比较
    [[nodiscard]] constexpr Quaternion operator-() const { return Quaternion(-x, -y, -z, -w); }
    [[nodiscard]] constexpr bool operator==(const Quaternion& other) const {
        return Math::abs(x - other.x) < Math::EPSILON && Math::abs(y - other.y) < Math::EPSILON &&
               Math::abs(z - other.z) < Math::EPSILON && Math::abs(w - other.w) < Math::EPSILON;
    }
    [[nodiscard]] constexpr bool operator!=(const Quaternion& other) const { return !(*this == other); }

    // 长度相关
    [[nodiscard]] constexpr float lengthSquared() const { return x * x + y * y + z * z + w * w; }
    [[nodiscard]] constexpr float magnitudeSquared() const { return lengthSquared(); }
    [[nodiscard]] float length() const;
    [[nodiscard]] float magnitude() const { return length(); }
    // 归一化
    Quaternion& normalize();
    [[nodiscard]] constexpr Quaternion normalized() const {
        float len = Math::sqrt(lengthSquared());
        return (len > Math::EPSILON) ? Quaternion(x / len, y / len, z / len, w / len) : *this;
    }
    [[nodiscard]] constexpr Quaternion unit() const { return normalized(); }
    // 共轭，xyz取反
    [[nodiscard]] constexpr Quaternion conjugate() const { return Quaternion(-x, -y, -z, w); }
    [[nodiscard]] constexpr Quaternion conjugated() const { return conjugate(); }
    // 逆，单位四元数的逆就是共轭
    [[nodiscard]] Quaternion inverse() const;
    Quaternion& invert();
    // 点积
    [[nodiscard]] constexpr float dot(const Quaternion& other) const {
        return x * other.x + y * other.y + z * other.z + w * other.w;
    }
    // 是不是单位四元数
    [[nodiscard]] constexpr bool isUnit() const {
        return Math::abs(lengthSquared() - 1.0f) < Math::EPSILON;
    }

    // 旋转向量
    [[nodiscard]] Vector3 rotate(const Vector3& vector) const;
    // slerp，球面线性插值，旋转平滑用这个
    [[nodiscard]] Quaternion slerp(const Quaternion& other, float t) const;
    // 快速slerp，用nlerp近似
    [[nodiscard]] Quaternion fastSlerp(const Quaternion& other, float t) const;
    // nlerp，归一化线性插值，快但不够平滑
    [[nodiscard]] constexpr Quaternion nlerp(const Quaternion& other, float t, bool shortestPath = true) const {
        float dotProduct = dot(other);
        Quaternion otherAdj = other;
        if (shortestPath && dotProduct < 0.0f) {
            otherAdj = -other;
            dotProduct = -dotProduct;
        }
        return Quaternion(
            x + otherAdj.x * t,
            y + otherAdj.y * t,
            z + otherAdj.z * t,
            w + otherAdj.w * t
        ).normalized();
    }
    [[nodiscard]] constexpr Quaternion lerp(const Quaternion& other, float t) const {
        return nlerp(other, t, true);
    }
    // 平方
    [[nodiscard]] constexpr Quaternion squared() const {
        return *this * *this;
    }
    // 幂运算
    [[nodiscard]] constexpr Quaternion pow(float exponent) const {
        if (Math::abs(w) > 0.9999f) return *this;
        float angle = Math::acos(w);
        float newAngle = angle * exponent;
        float sinNewAngle = Math::sin(newAngle);
        float s = Math::sqrt(1.0f - w * w);
        if (s < Math::EPSILON) return Quaternion::identity;
        return Quaternion(
            x / s * sinNewAngle,
            y / s * sinNewAngle,
            z / s * sinNewAngle,
            Math::cos(newAngle)
        );
    }
    // 对数和指数，高级插值会用
    [[nodiscard]] Quaternion log() const {
        float len = Math::sqrt(x * x + y * y + z * z);
        if (len < Math::EPSILON) return Quaternion(0.0f, 0.0f, 0.0f, Math::log(w));
        float angle = Math::atan2(len, w);
        float factor = angle / len;
        return Quaternion(x * factor, y * factor, z * factor, Math::log(length()));
    }
    [[nodiscard]] Quaternion exp() const {
        float theta = Math::sqrt(x * x + y * y + z * z);
        if (theta < Math::EPSILON) return Quaternion(x, y, z, Math::exp(w));
        float sinTheta = Math::sin(theta);
        float factor = sinTheta / theta;
        float magnitude = Math::exp(w);
        return Quaternion(x * factor * magnitude, y * factor * magnitude, z * factor * magnitude, Math::cos(theta) * magnitude);
    }
    // 获取角度
    [[nodiscard]] float getAngleDegrees() const { return getAngle(true); }
    // 欧拉角
    [[nodiscard]] Vector3 getEulerAngles(bool isDegrees = false) const { return toEulerAngles(isDegrees); }
    [[nodiscard]] Vector3 getEulerAnglesDegrees() const { return toEulerAngles(true); }
    // 从两个方向构造旋转
    [[nodiscard]] static Quaternion fromToRotation(const Vector3& fromDirection, const Vector3& toDirection);
    // 朝目标旋转
    [[nodiscard]] static Quaternion rotateTowards(const Quaternion& from, const Quaternion& to, float maxDegreesDelta);
    // 轴角
    [[nodiscard]] static Quaternion angleAxis(float angle, const Vector3& axis, bool isDegrees = false) {
        return fromAxisAngle(axis, angle, isDegrees);
    }
    // 欧拉角构造
    [[nodiscard]] static Quaternion euler(float xAngle, float yAngle, float zAngle, bool isDegrees = false) {
        return fromEulerAngles(Vector3(xAngle, yAngle, zAngle), isDegrees);
    }

    // 转换方法
    [[nodiscard]] Matrix3x3 toRotationMatrix() const;
    [[nodiscard]] Matrix4x4 toRotationMatrix4x4() const;
    // 转欧拉角
    [[nodiscard]] Vector3 toEulerAngles(bool isDegrees = false) const;
    // 获取旋转轴
    constexpr Vector3 getAxis() const {
        float s = Math::sqrt(1.0f - w * w);
        return (s > Math::EPSILON) ? Vector3(x / s, y / s, z / s) : Vector3::forward;
    }
    float getAngle(bool isDegrees = false) const;

    // 常量，直接用
    static const Quaternion identity;
    static const Quaternion zero;

    // 静态方法
    static constexpr Quaternion Identity() { return Quaternion(0.0f, 0.0f, 0.0f, 1.0f); }
    static constexpr Quaternion Zero() { return Quaternion(0.0f, 0.0f, 0.0f, 0.0f); }
    static Quaternion fromAxisAngle(const Vector3& axis, float angle, bool isDegrees = false);
    static Quaternion fromEulerAngles(const Vector3& eulerAngles, bool isDegrees = false);
    static Quaternion fromRotationMatrix(const Matrix3x3& rotationMatrix);
    static Quaternion fromRotationMatrix(const Matrix4x4& rotationMatrix);
    // 看向某个方向
    static Quaternion lookRotation(const Vector3& forward, const Vector3& up = Vector3::up);
    static constexpr float dot(const Quaternion& a, const Quaternion& b) {
        return a.x * b.x + a.y * b.y + a.z * b.z + a.w * b.w;
    }
    static Quaternion slerp(const Quaternion& a, const Quaternion& b, float t);
    static Quaternion fastSlerp(const Quaternion& a, const Quaternion& b, float t);
    static Quaternion nlerp(const Quaternion& a, const Quaternion& b, float t, bool shortestPath = true);
    // 两个四元数之间的角度
    static constexpr float angle(const Quaternion& a, const Quaternion& b) {
        float dot = Math::abs(a.dot(b));
        return (dot > 1.0f) ? 0.0f : (Math::acos(dot) * 2.0f);
    }
    static constexpr float angleDegrees(const Quaternion& a, const Quaternion& b) {
        return Math::radToDeg(angle(a, b));
    }

    // 调试输出
    friend std::ostream& operator<<(std::ostream& os, const Quaternion& quat);
};

// 全局操作符，让 scalar * quat 也能用
constexpr Quaternion operator*(float scalar, const Quaternion& quat) { return quat * scalar; }
constexpr Quaternion operator+(float scalar, const Quaternion& quat) {
    return Quaternion(scalar + quat.x, scalar + quat.y, scalar + quat.z, scalar + quat.w);
}
constexpr Quaternion operator+(const Quaternion& quat, float scalar) {
    return Quaternion(quat.x + scalar, quat.y + scalar, quat.z + scalar, quat.w + scalar);
}
constexpr Quaternion operator-(const Quaternion& quat, float scalar) {
    return Quaternion(quat.x - scalar, quat.y - scalar, quat.z - scalar, quat.w - scalar);
}
constexpr Quaternion operator/(float scalar, const Quaternion& quat) {
    return Quaternion(scalar / quat.x, scalar / quat.y, scalar / quat.z, scalar / quat.w);
}

} // namespace AstraAlgebra
