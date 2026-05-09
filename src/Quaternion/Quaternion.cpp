// Quaternion.cpp - 四元数类实现
// 3D旋转用的，比欧拉角好使，不会万向节死锁

#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

namespace AstraAlgebra {

// 初始化静态常量
const Quaternion Quaternion::identity = Quaternion(0.0f, 0.0f, 0.0f, 1.0f);
const Quaternion Quaternion::zero = Quaternion(0.0f, 0.0f, 0.0f, 0.0f);

// 构造函数
Quaternion::Quaternion(const Vector3& axis, float angle, bool isDegrees) {
    if (isDegrees) angle = Math::degToRad(angle);
    float halfAngle = angle * 0.5f;
    float sinHalfAngle = std::sin(halfAngle);
    Vector3 normalizedAxis = axis.normalized();
    x = normalizedAxis.x * sinHalfAngle;
    y = normalizedAxis.y * sinHalfAngle;
    z = normalizedAxis.z * sinHalfAngle;
    w = std::cos(halfAngle);
}

Quaternion::Quaternion(const Vector3& eulerAngles, bool isDegrees) {
    float roll = isDegrees ? Math::degToRad(eulerAngles.z) : eulerAngles.z;
    float pitch = isDegrees ? Math::degToRad(eulerAngles.x) : eulerAngles.x;
    float yaw = isDegrees ? Math::degToRad(eulerAngles.y) : eulerAngles.y;
    float cy = std::cos(yaw * 0.5f), sy = std::sin(yaw * 0.5f);
    float cp = std::cos(pitch * 0.5f), sp = std::sin(pitch * 0.5f);
    float cr = std::cos(roll * 0.5f), sr = std::sin(roll * 0.5f);
    w = cy * cp * cr + sy * sp * sr;
    x = cy * cp * sr - sy * sp * cr;
    y = sy * cp * sr + cy * sp * cr;
    z = sy * cp * cr - cy * sp * sr;
}

Quaternion::Quaternion(const Matrix3x3& rotationMatrix) {
    float trace = rotationMatrix.m[0][0] + rotationMatrix.m[1][1] + rotationMatrix.m[2][2];
    if (trace > 0.0f) {
        float s = 0.5f / std::sqrt(trace + 1.0f);
        w = 0.25f / s;
        x = (rotationMatrix.m[2][1] - rotationMatrix.m[1][2]) * s;
        y = (rotationMatrix.m[0][2] - rotationMatrix.m[2][0]) * s;
        z = (rotationMatrix.m[1][0] - rotationMatrix.m[0][1]) * s;
    } else {
        if (rotationMatrix.m[0][0] > rotationMatrix.m[1][1] && rotationMatrix.m[0][0] > rotationMatrix.m[2][2]) {
            float s = 2.0f * std::sqrt(1.0f + rotationMatrix.m[0][0] - rotationMatrix.m[1][1] - rotationMatrix.m[2][2]);
            w = (rotationMatrix.m[2][1] - rotationMatrix.m[1][2]) / s;
            x = 0.25f * s;
            y = (rotationMatrix.m[0][1] + rotationMatrix.m[1][0]) / s;
            z = (rotationMatrix.m[0][2] + rotationMatrix.m[2][0]) / s;
        } else if (rotationMatrix.m[1][1] > rotationMatrix.m[2][2]) {
            float s = 2.0f * std::sqrt(1.0f + rotationMatrix.m[1][1] - rotationMatrix.m[0][0] - rotationMatrix.m[2][2]);
            w = (rotationMatrix.m[0][2] - rotationMatrix.m[2][0]) / s;
            x = (rotationMatrix.m[0][1] + rotationMatrix.m[1][0]) / s;
            y = 0.25f * s;
            z = (rotationMatrix.m[1][2] + rotationMatrix.m[2][1]) / s;
        } else {
            float s = 2.0f * std::sqrt(1.0f + rotationMatrix.m[2][2] - rotationMatrix.m[0][0] - rotationMatrix.m[1][1]);
            w = (rotationMatrix.m[1][0] - rotationMatrix.m[0][1]) / s;
            x = (rotationMatrix.m[0][2] + rotationMatrix.m[2][0]) / s;
            y = (rotationMatrix.m[1][2] + rotationMatrix.m[2][1]) / s;
            z = 0.25f * s;
        }
    }
}

Quaternion::Quaternion(const Matrix4x4& rotationMatrix) {
    *this = Quaternion(rotationMatrix.getRotationMatrix());
}

// 赋值操作符
Quaternion& Quaternion::operator=(const Quaternion& other) {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
    }
    return *this;
}

Quaternion& Quaternion::operator=(Quaternion&& other) noexcept {
    if (this != &other) {
        x = other.x;
        y = other.y;
        z = other.z;
        w = other.w;
        other.x = 0.0f; other.y = 0.0f; other.z = 0.0f; other.w = 0.0f;
    }
    return *this;
}

// 复合赋值操作符
Quaternion& Quaternion::operator+=(const Quaternion& other) {
    x += other.x; y += other.y; z += other.z; w += other.w;
    return *this;
}

Quaternion& Quaternion::operator-=(const Quaternion& other) {
    x -= other.x; y -= other.y; z -= other.z; w -= other.w;
    return *this;
}

Quaternion& Quaternion::operator*=(const Quaternion& other) {
    *this = *this * other;
    return *this;
}

Quaternion& Quaternion::operator*=(float scalar) {
    x *= scalar; y *= scalar; z *= scalar; w *= scalar;
    return *this;
}

Quaternion& Quaternion::operator/=(float scalar) {
    float inv = 1.0f / scalar;
    x *= inv; y *= inv; z *= inv; w *= inv;
    return *this;
}

// 四元数基本操作
float Quaternion::length() const {
    return Math::sqrt(lengthSquared());
}

Quaternion& Quaternion::normalize() {
    float len = length();
    if (len > Math::EPSILON) {
        x /= len; y /= len; z /= len; w /= len;
    }
    return *this;
}

Quaternion Quaternion::inverse() const {
    float lenSq = lengthSquared();
    if (lenSq < Math::EPSILON) return Quaternion::identity;
    Quaternion conj = conjugate();
    return conj / lenSq;
}

Quaternion& Quaternion::invert() {
    *this = inverse();
    return *this;
}

// 旋转操作
Vector3 Quaternion::rotate(const Vector3& vector) const {
    return *this * vector;
}

Quaternion Quaternion::slerp(const Quaternion& other, float t) const {
    float dotProduct = x * other.x + y * other.y + z * other.z + w * other.w;
    float sign = (dotProduct < 0.0f) ? -1.0f : 1.0f;
    dotProduct *= sign;
    
    if (dotProduct > 0.9995f) {
        float t1 = 1.0f - t;
        float t2 = t * sign;
        return Quaternion(
            x * t1 + other.x * t2,
            y * t1 + other.y * t2,
            z * t1 + other.z * t2,
            w * t1 + other.w * t2
        ).normalized();
    }
    
    float angle = std::acos(Math::clamp(dotProduct, -1.0f, 1.0f));
    float sinAngle = std::sin(angle);
    
    if (Math::abs(sinAngle) < Math::EPSILON) {
        float t1 = 1.0f - t;
        float t2 = t * sign;
        return Quaternion(
            x * t1 + other.x * t2,
            y * t1 + other.y * t2,
            z * t1 + other.z * t2,
            w * t1 + other.w * t2
        ).normalized();
    }
    
    float invSinAngle = 1.0f / sinAngle;
    float t1 = std::sin((1.0f - t) * angle) * invSinAngle;
    float t2 = std::sin(t * angle) * invSinAngle * sign;
    
    return Quaternion(
        x * t1 + other.x * t2,
        y * t1 + other.y * t2,
        z * t1 + other.z * t2,
        w * t1 + other.w * t2
    );
}

Quaternion Quaternion::fastSlerp(const Quaternion& other, float t) const {
    float dotProduct = dot(other);
    Quaternion otherAdj = other;
    
    if (dotProduct < 0.0f) {
        otherAdj = -other;
        dotProduct = -dotProduct;
    }
    
    if (dotProduct > 0.9995f) {
        return nlerp(otherAdj, t, false);
    }
    
    float theta = std::acos(dotProduct);
    float thetaT = t * theta;
    
    float sinTheta = std::sin(theta);
    float sinThetaT = std::sin(thetaT);
    
    float invSinTheta = 1.0f / sinTheta;
    float ratio = sinThetaT * invSinTheta;
    float ratio0 = std::cos(thetaT) - dotProduct * ratio;
    
    return Quaternion(
        ratio0 * x + ratio * otherAdj.x,
        ratio0 * y + ratio * otherAdj.y,
        ratio0 * z + ratio * otherAdj.z,
        ratio0 * w + ratio * otherAdj.w
    );
}

// 转换方法
Matrix3x3 Quaternion::toRotationMatrix() const {
    float xx = x * x, yy = y * y, zz = z * z;
    float xy = x * y, xz = x * z, yz = y * z;
    float wx = w * x, wy = w * y, wz = w * z;
    
    return Matrix3x3(
        1.0f - 2.0f * (yy + zz), 2.0f * (xy - wz), 2.0f * (xz + wy),
        2.0f * (xy + wz), 1.0f - 2.0f * (xx + zz), 2.0f * (yz - wx),
        2.0f * (xz - wy), 2.0f * (yz + wx), 1.0f - 2.0f * (xx + yy)
    );
}

Matrix4x4 Quaternion::toRotationMatrix4x4() const {
    return Matrix4x4::fromRotationMatrix(toRotationMatrix());
}

Vector3 Quaternion::toEulerAngles(bool isDegrees) const {
    float sinr_cosp = 2.0f * (w * x + y * z);
    float cosr_cosp = 1.0f - 2.0f * (x * x + y * y);
    float roll = Math::atan2(sinr_cosp, cosr_cosp);
    
    float sinp = 2.0f * (w * y - z * x);
    float pitch;
    if (Math::abs(sinp) >= 1.0f) {
        pitch = (sinp >= 0 ? 1.0f : -1.0f) * Math::PI / 2.0f;
    } else {
        pitch = Math::asin(sinp);
    }
    
    float siny_cosp = 2.0f * (w * z + x * y);
    float cosy_cosp = 1.0f - 2.0f * (y * y + z * z);
    float yaw = Math::atan2(siny_cosp, cosy_cosp);
    
    Vector3 result(pitch, yaw, roll);
    if (isDegrees) {
        result = result * (180.0f / Math::PI);
    }
    return result;
}

float Quaternion::getAngle(bool isDegrees) const {
    float angle = 2.0f * Math::acos(Math::clamp(w, -1.0f, 1.0f));
    return isDegrees ? Math::radToDeg(angle) : angle;
}

// 静态方法
Quaternion Quaternion::fromAxisAngle(const Vector3& axis, float angle, bool isDegrees) {
    return Quaternion(axis, angle, isDegrees);
}

Quaternion Quaternion::fromEulerAngles(const Vector3& eulerAngles, bool isDegrees) {
    return Quaternion(eulerAngles, isDegrees);
}

Quaternion Quaternion::fromRotationMatrix(const Matrix3x3& rotationMatrix) {
    return Quaternion(rotationMatrix);
}

Quaternion Quaternion::fromRotationMatrix(const Matrix4x4& rotationMatrix) {
    return Quaternion(rotationMatrix);
}

Quaternion Quaternion::lookRotation(const Vector3& forward, const Vector3& up) {
    Vector3 f = forward.normalized();
    Vector3 r = f.cross(up).normalized();
    Vector3 u = r.cross(f);
    
    Matrix3x3 m(r.x, r.y, r.z, u.x, u.y, u.z, -f.x, -f.y, -f.z);
    return Quaternion(m);
}

Quaternion Quaternion::slerp(const Quaternion& a, const Quaternion& b, float t) {
    return a.slerp(b, t);
}

Quaternion Quaternion::fastSlerp(const Quaternion& a, const Quaternion& b, float t) {
    return a.fastSlerp(b, t);
}

Quaternion Quaternion::nlerp(const Quaternion& a, const Quaternion& b, float t, bool shortestPath) {
    return a.nlerp(b, t, shortestPath);
}

Quaternion Quaternion::fromToRotation(const Vector3& fromDirection, const Vector3& toDirection) {
    Vector3 from = fromDirection.normalized();
    Vector3 to = toDirection.normalized();
    float dotProduct = from.dot(to);
    if (dotProduct > 0.99999f) return Quaternion::identity;
    if (dotProduct < -0.99999f) {
        Vector3 axis = Vector3::up.cross(from);
        if (axis.lengthSquared() < 0.0001f) {
            axis = Vector3::right.cross(from);
        }
        return Quaternion(axis.normalized(), 180.0f, true);
    }
    Vector3 axis = from.cross(to).normalized();
    float angle = Math::acos(Math::clamp(dotProduct, -1.0f, 1.0f));
    return Quaternion(axis, angle, false);
}

Quaternion Quaternion::rotateTowards(const Quaternion& from, const Quaternion& to, float maxDegreesDelta) {
    float angle = Quaternion::angle(from, to);
    if (angle < 0.0001f) return to;
    float t = Math::min(1.0f, maxDegreesDelta / angle);
    return from.slerp(to, t);
}

// 调试输出
std::ostream& operator<<(std::ostream& os, const Quaternion& quat) {
    os << "(" << quat.x << ", " << quat.y << ", " << quat.z << ", " << quat.w << ")";
    return os;
}

} // namespace AstraAlgebra
