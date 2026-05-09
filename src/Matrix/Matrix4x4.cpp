// Matrix4x4.cpp - 4x4矩阵类实现
// 3D变换的核心，平移旋转缩放投影都离不开它

#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

// 添加SIMD支持的头文件
#ifdef __AVX2__
#include <immintrin.h>
#elif defined(__AVX__)
#include <immintrin.h>
#elif defined(__SSE2__)
#include <emmintrin.h>
#elif defined(_MSC_VER)
#include <immintrin.h>
#endif

namespace AstraAlgebra {

// 初始化静态常量
const Matrix4x4 Matrix4x4::identity = Matrix4x4(1.0f, 0.0f, 0.0f, 0.0f,
                                                0.0f, 1.0f, 0.0f, 0.0f,
                                                0.0f, 0.0f, 1.0f, 0.0f,
                                                0.0f, 0.0f, 0.0f, 1.0f);
const Matrix4x4 Matrix4x4::zero = Matrix4x4(0.0f, 0.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 0.0f, 0.0f,
                                            0.0f, 0.0f, 0.0f, 0.0f);

// 赋值操作符
Matrix4x4& Matrix4x4::operator=(const Matrix4x4& other) {
    if (this != &other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = other.m[i][j];
            }
        }
    }
    return *this;
}

Matrix4x4& Matrix4x4::operator=(Matrix4x4&& other) noexcept {
    if (this != &other) {
        for (int i = 0; i < 4; ++i) {
            for (int j = 0; j < 4; ++j) {
                m[i][j] = other.m[i][j];
                other.m[i][j] = 0.0f;
            }
        }
    }
    return *this;
}

// 复合赋值操作符
Matrix4x4& Matrix4x4::operator+=(const Matrix4x4& other) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m[i][j] += other.m[i][j];
    return *this;
}

Matrix4x4& Matrix4x4::operator-=(const Matrix4x4& other) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m[i][j] -= other.m[i][j];
    return *this;
}

Matrix4x4& Matrix4x4::operator*=(const Matrix4x4& other) {
    Matrix4x4 temp = *this * other;
    *this = temp;
    return *this;
}

Matrix4x4& Matrix4x4::operator*=(float scalar) {
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m[i][j] *= scalar;
    return *this;
}

Matrix4x4& Matrix4x4::operator/=(float scalar) {
    float invScalar = 1.0f / scalar;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            m[i][j] *= invScalar;
    return *this;
}

// 矩阵操作方法
Matrix4x4& Matrix4x4::transposeInPlace() {
    *this = transposed();
    return *this;
}

float Matrix4x4::determinant() const {
    float det = 0.0f;
    for (int i = 0; i < 4; ++i) {
        float minor[3][3];
        int mi = 0;
        for (int j = 0; j < 4; ++j) {
            if (j == i) continue;
            int mj = 0;
            for (int k = 1; k < 4; ++k) {
                minor[mi][mj] = m[j][k];
                mj++;
            }
            mi++;
        }
        float minorDet = minor[0][0] * (minor[1][1] * minor[2][2] - minor[1][2] * minor[2][1])
                       - minor[0][1] * (minor[1][0] * minor[2][2] - minor[1][2] * minor[2][0])
                       + minor[0][2] * (minor[1][0] * minor[2][1] - minor[1][1] * minor[2][0]);
        det += (i % 2 == 0 ? 1 : -1) * m[i][0] * minorDet;
    }
    return det;
}

Matrix4x4 Matrix4x4::inverse() const {
    // 快速路径：检查是否是纯旋转矩阵（正交矩阵 + 行列式为 1）
    if (m[0][3] == 0.0f && m[1][3] == 0.0f && m[2][3] == 0.0f && m[3][3] == 1.0f &&
        m[3][0] == 0.0f && m[3][1] == 0.0f && m[3][2] == 0.0f) {
        // 检查是否是正交矩阵（R^T * R = I）
        float dot01 = m[0][0]*m[1][0] + m[0][1]*m[1][1] + m[0][2]*m[1][2];
        float dot02 = m[0][0]*m[2][0] + m[0][1]*m[2][1] + m[0][2]*m[2][2];
        float dot12 = m[1][0]*m[2][0] + m[1][1]*m[2][1] + m[1][2]*m[2][2];
        float len0 = m[0][0]*m[0][0] + m[0][1]*m[0][1] + m[0][2]*m[0][2];
        float len1 = m[1][0]*m[1][0] + m[1][1]*m[1][1] + m[1][2]*m[1][2];
        float len2 = m[2][0]*m[2][0] + m[2][1]*m[2][1] + m[2][2]*m[2][2];
        
        if (Math::abs(dot01) < Math::EPSILON && Math::abs(dot02) < Math::EPSILON && 
            Math::abs(dot12) < Math::EPSILON && 
            Math::abs(len0 - 1.0f) < Math::EPSILON && 
            Math::abs(len1 - 1.0f) < Math::EPSILON && 
            Math::abs(len2 - 1.0f) < Math::EPSILON) {
            // 旋转矩阵的逆就是转置
            return Matrix4x4(
                m[0][0], m[1][0], m[2][0], 0.0f,
                m[0][1], m[1][1], m[2][1], 0.0f,
                m[0][2], m[1][2], m[2][2], 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f
            );
        }
    }
    
    // 使用 double 进行中间计算，避免极端值溢出
    double a00 = m[0][0], a01 = m[0][1], a02 = m[0][2], a03 = m[0][3];
    double a10 = m[1][0], a11 = m[1][1], a12 = m[1][2], a13 = m[1][3];
    double a20 = m[2][0], a21 = m[2][1], a22 = m[2][2], a23 = m[2][3];
    double a30 = m[3][0], a31 = m[3][1], a32 = m[3][2], a33 = m[3][3];
    
    double b00 = a00*a11 - a01*a10;
    double b01 = a00*a12 - a02*a10;
    double b02 = a00*a13 - a03*a10;
    double b03 = a01*a12 - a02*a11;
    double b04 = a01*a13 - a03*a11;
    double b05 = a02*a13 - a03*a12;
    double b06 = a20*a31 - a21*a30;
    double b07 = a20*a32 - a22*a30;
    double b08 = a20*a33 - a23*a30;
    double b09 = a21*a32 - a22*a31;
    double b10 = a21*a33 - a23*a31;
    double b11 = a22*a33 - a23*a32;
    
    double det = b00*b11 - b01*b10 + b02*b09 + b03*b08 - b04*b07 + b05*b06;
    
    // 检查行列式是否为零、无穷大或NaN
    if (Math::abs(det) < Math::EPSILON || std::isinf(det) || std::isnan(det)) {
        return Matrix4x4::identity;
    }
    
    double inv_det = 1.0 / det;
    
    // 计算逆矩阵元素
    double r00 = (a11*b11 - a12*b10 + a13*b09) * inv_det;
    double r01 = (a02*b10 - a01*b11 - a03*b09) * inv_det;
    double r02 = (a31*b05 - a32*b04 + a33*b03) * inv_det;
    double r03 = (a22*b04 - a21*b05 - a23*b03) * inv_det;
    
    double r10 = (a12*b08 - a10*b11 - a13*b07) * inv_det;
    double r11 = (a00*b11 - a02*b08 + a03*b07) * inv_det;
    double r12 = (a32*b02 - a30*b05 - a33*b01) * inv_det;
    double r13 = (a20*b05 - a22*b02 + a23*b01) * inv_det;
    
    double r20 = (a10*b10 - a11*b08 + a13*b06) * inv_det;
    double r21 = (a01*b08 - a00*b10 - a03*b06) * inv_det;
    double r22 = (a30*b04 - a31*b02 + a33*b00) * inv_det;
    double r23 = (a21*b02 - a20*b04 - a23*b00) * inv_det;
    
    double r30 = (a11*b07 - a10*b09 - a12*b06) * inv_det;
    double r31 = (a00*b09 - a01*b07 + a02*b06) * inv_det;
    double r32 = (a31*b01 - a30*b03 - a32*b00) * inv_det;
    double r33 = (a20*b03 - a21*b01 + a22*b00) * inv_det;
    
    // 如果任何元素是NaN或无穷大，返回单位矩阵
    if (std::isnan(r00) || std::isnan(r01) || std::isnan(r02) || std::isnan(r03) ||
        std::isnan(r10) || std::isnan(r11) || std::isnan(r12) || std::isnan(r13) ||
        std::isnan(r20) || std::isnan(r21) || std::isnan(r22) || std::isnan(r23) ||
        std::isnan(r30) || std::isnan(r31) || std::isnan(r32) || std::isnan(r33) ||
        std::isinf(r00) || std::isinf(r01) || std::isinf(r02) || std::isinf(r03) ||
        std::isinf(r10) || std::isinf(r11) || std::isinf(r12) || std::isinf(r13) ||
        std::isinf(r20) || std::isinf(r21) || std::isinf(r22) || std::isinf(r23) ||
        std::isinf(r30) || std::isinf(r31) || std::isinf(r32) || std::isinf(r33)) {
        return Matrix4x4::identity;
    }
    
    return Matrix4x4(
        static_cast<float>(r00), static_cast<float>(r01), static_cast<float>(r02), static_cast<float>(r03),
        static_cast<float>(r10), static_cast<float>(r11), static_cast<float>(r12), static_cast<float>(r13),
        static_cast<float>(r20), static_cast<float>(r21), static_cast<float>(r22), static_cast<float>(r23),
        static_cast<float>(r30), static_cast<float>(r31), static_cast<float>(r32), static_cast<float>(r33)
    );
}

Matrix4x4& Matrix4x4::invert() {
    *this = inverse();
    return *this;
}

// 分解变换矩阵
void Matrix4x4::setTranslation(const Vector3& translation) {
    m[0][3] = translation.x;
    m[1][3] = translation.y;
    m[2][3] = translation.z;
}

Matrix3x3 Matrix4x4::getRotationMatrix() const {
    return Matrix3x3(
        m[0][0], m[0][1], m[0][2],
        m[1][0], m[1][1], m[1][2],
        m[2][0], m[2][1], m[2][2]
    );
}

Vector3 Matrix4x4::getScale() const {
    return Vector3(
        Math::sqrt(m[0][0] * m[0][0] + m[0][1] * m[0][1] + m[0][2] * m[0][2]),
        Math::sqrt(m[1][0] * m[1][0] + m[1][1] * m[1][1] + m[1][2] * m[1][2]),
        Math::sqrt(m[2][0] * m[2][0] + m[2][1] * m[2][1] + m[2][2] * m[2][2])
    );
}

void Matrix4x4::decompose(Vector3& position, Quaternion& rotation, Vector3& scale) const {
    position = getTranslation();
    scale = getScale();
    
    Matrix3x3 rotMat = getRotationMatrix();
    for (int j = 0; j < 3; ++j) rotMat[0][j] /= scale.x;
    for (int j = 0; j < 3; ++j) rotMat[1][j] /= scale.y;
    for (int j = 0; j < 3; ++j) rotMat[2][j] /= scale.z;
    
    rotation = Quaternion(rotMat);
}

// 静态方法
Matrix4x4 Matrix4x4::rotationX(float angle, bool isDegrees) {
    float a = isDegrees ? Math::degToRad(angle) : angle;
    float c = Math::cos(a);
    float s = Math::sin(a);
    return Matrix4x4(
        1.0f, 0.0f, 0.0f, 0.0f,
        0.0f, c, -s, 0.0f,
        0.0f, s, c, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::rotationY(float angle, bool isDegrees) {
    float a = isDegrees ? Math::degToRad(angle) : angle;
    float c = Math::cos(a);
    float s = Math::sin(a);
    return Matrix4x4(
        c, 0.0f, s, 0.0f,
        0.0f, 1.0f, 0.0f, 0.0f,
        -s, 0.0f, c, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::rotationZ(float angle, bool isDegrees) {
    float a = isDegrees ? Math::degToRad(angle) : angle;
    float c = Math::cos(a);
    float s = Math::sin(a);
    return Matrix4x4(
        c, -s, 0.0f, 0.0f,
        s, c, 0.0f, 0.0f,
        0.0f, 0.0f, 1.0f, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::rotation(const Vector3& axis, float angle, bool isDegrees) {
    float a = isDegrees ? Math::degToRad(angle) : angle;
    Vector3 a_norm = axis.normalized();
    float c = Math::cos(a);
    float s = Math::sin(a);
    float t = 1.0f - c;
    
    return Matrix4x4(
        t * a_norm.x * a_norm.x + c, t * a_norm.x * a_norm.y - s * a_norm.z, t * a_norm.x * a_norm.z + s * a_norm.y, 0.0f,
        t * a_norm.x * a_norm.y + s * a_norm.z, t * a_norm.y * a_norm.y + c, t * a_norm.y * a_norm.z - s * a_norm.x, 0.0f,
        t * a_norm.x * a_norm.z - s * a_norm.y, t * a_norm.y * a_norm.z + s * a_norm.x, t * a_norm.z * a_norm.z + c, 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::rotation(const Vector3& eulerAngles, bool isDegrees) {
    return rotationZ(eulerAngles.z, isDegrees) * rotationY(eulerAngles.y, isDegrees) * rotationX(eulerAngles.x, isDegrees);
}

Matrix4x4 Matrix4x4::lookAt(const Vector3& eye, const Vector3& target, const Vector3& up) {
    float fx = target.x - eye.x;
    float fy = target.y - eye.y;
    float fz = target.z - eye.z;
    float fLen = std::sqrt(fx * fx + fy * fy + fz * fz);
    float fInvLen = (fLen > Math::EPSILON) ? 1.0f / fLen : 0.0f;
    fx *= fInvLen; fy *= fInvLen; fz *= fInvLen;
    
    float rx = fy * up.z - fz * up.y;
    float ry = fz * up.x - fx * up.z;
    float rz = fx * up.y - fy * up.x;
    float rLen = std::sqrt(rx * rx + ry * ry + rz * rz);
    float rInvLen = (rLen > Math::EPSILON) ? 1.0f / rLen : 0.0f;
    rx *= rInvLen; ry *= rInvLen; rz *= rInvLen;
    
    float ux = ry * fz - rz * fy;
    float uy = rz * fx - rx * fz;
    float uz = rx * fy - ry * fx;
    
    return Matrix4x4(
        rx, ry, rz, -(rx * eye.x + ry * eye.y + rz * eye.z),
        ux, uy, uz, -(ux * eye.x + uy * eye.y + uz * eye.z),
        -fx, -fy, -fz, fx * eye.x + fy * eye.y + fz * eye.z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::perspective(float fov, float aspectRatio, float nearPlane, float farPlane) {
    float fovRad = fov * Math::DEG_TO_RAD;
    float f = 1.0f / tanf(fovRad * 0.5f);
    float nf = 1.0f / (nearPlane - farPlane);
    
    return Matrix4x4(
        f / aspectRatio, 0.0f, 0.0f, 0.0f,
        0.0f, f, 0.0f, 0.0f,
        0.0f, 0.0f, (farPlane + nearPlane) * nf, 2.0f * farPlane * nearPlane * nf,
        0.0f, 0.0f, -1.0f, 0.0f
    );
}

Matrix4x4 Matrix4x4::orthographic(float left, float right, float bottom, float top, float nearPlane, float farPlane) {
    float lr = 1.0f / (left - right);
    float bt = 1.0f / (bottom - top);
    float nf = 1.0f / (nearPlane - farPlane);
    
    return Matrix4x4(
        -2.0f * lr, 0.0f, 0.0f, (left + right) * lr,
        0.0f, -2.0f * bt, 0.0f, (bottom + top) * bt,
        0.0f, 0.0f, 2.0f * nf, (farPlane + nearPlane) * nf,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::fromRotationMatrix(const Matrix3x3& rotation) {
    return Matrix4x4(
        rotation.m[0][0], rotation.m[0][1], rotation.m[0][2], 0.0f,
        rotation.m[1][0], rotation.m[1][1], rotation.m[1][2], 0.0f,
        rotation.m[2][0], rotation.m[2][1], rotation.m[2][2], 0.0f,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::fromTRS(const Vector3& translation, const Quaternion& rotation, const Vector3& scale) {
    Matrix4x4 result = Matrix4x4::identity;
    Matrix3x3 rotMat = rotation.toRotationMatrix();
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            result.m[i][j] = rotMat.m[i][j] * scale[j];
        }
    }
    result.m[0][3] = translation.x;
    result.m[1][3] = translation.y;
    result.m[2][3] = translation.z;
    return result;
}

Matrix4x4 Matrix4x4::perspectiveFov(float fovY, float aspect, float zNear, float zFar, bool isDegrees) {
    float fov = isDegrees ? fovY * Math::DEG_TO_RAD : fovY;
    float f = 1.0f / tanf(fov * 0.5f);
    float nf = 1.0f / (zNear - zFar);
    return Matrix4x4(
        f / aspect, 0.0f, 0.0f, 0.0f,
        0.0f, f, 0.0f, 0.0f,
        0.0f, 0.0f, (zFar + zNear) * nf, 2.0f * zFar * zNear * nf,
        0.0f, 0.0f, -1.0f, 0.0f
    );
}

Matrix4x4 Matrix4x4::perspectiveOffCenter(float left, float right, float bottom, float top, float nearPlane, float farPlane) {
    float nf = 1.0f / (nearPlane - farPlane);
    return Matrix4x4(
        2.0f * nearPlane / (right - left), 0.0f, (right + left) / (right - left), 0.0f,
        0.0f, 2.0f * nearPlane / (top - bottom), (top + bottom) / (top - bottom), 0.0f,
        0.0f, 0.0f, (farPlane + nearPlane) * nf, 2.0f * farPlane * nearPlane * nf,
        0.0f, 0.0f, -1.0f, 0.0f
    );
}

Matrix4x4 Matrix4x4::ortho(float left, float right, float bottom, float top, float nearPlane, float farPlane) {
    return orthographic(left, right, bottom, top, nearPlane, farPlane);
}

Matrix4x4 Matrix4x4::lookTo(const Vector3& eye, const Vector3& forward, const Vector3& up) {
    float fx = forward.x, fy = forward.y, fz = forward.z;
    float fLen = std::sqrt(fx * fx + fy * fy + fz * fz);
    float fInvLen = (fLen > Math::EPSILON) ? 1.0f / fLen : 0.0f;
    fx *= fInvLen; fy *= fInvLen; fz *= fInvLen;
    
    float rx = fy * up.z - fz * up.y;
    float ry = fz * up.x - fx * up.z;
    float rz = fx * up.y - fy * up.x;
    float rLen = std::sqrt(rx * rx + ry * ry + rz * rz);
    float rInvLen = (rLen > Math::EPSILON) ? 1.0f / rLen : 0.0f;
    rx *= rInvLen; ry *= rInvLen; rz *= rInvLen;
    
    float ux = ry * fz - rz * fy;
    float uy = rz * fx - rx * fz;
    float uz = rx * fy - ry * fx;
    
    return Matrix4x4(
        rx, ry, rz, -(rx * eye.x + ry * eye.y + rz * eye.z),
        ux, uy, uz, -(ux * eye.x + uy * eye.y + uz * eye.z),
        -fx, -fy, -fz, fx * eye.x + fy * eye.y + fz * eye.z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::reflection(const Vector3& point, const Vector3& normal) {
    Vector3 n = normal.normalized();
    float d = -n.dot(point);
    return Matrix4x4(
        1.0f - 2.0f * n.x * n.x, -2.0f * n.x * n.y, -2.0f * n.x * n.z, -2.0f * d * n.x,
        -2.0f * n.y * n.x, 1.0f - 2.0f * n.y * n.y, -2.0f * n.y * n.z, -2.0f * d * n.y,
        -2.0f * n.z * n.x, -2.0f * n.z * n.y, 1.0f - 2.0f * n.z * n.z, -2.0f * d * n.z,
        0.0f, 0.0f, 0.0f, 1.0f
    );
}

Matrix4x4 Matrix4x4::shadow(const Vector3& lightPos, const Vector3& planeNormal, float planeD) {
    Vector3 n = planeNormal.normalized();
    float dotLN = n.dot(lightPos);
    float denom = dotLN - planeD;
    if (Math::abs(denom) < Math::EPSILON) return Matrix4x4::identity;
    float t = -denom;
    return Matrix4x4(
        t + lightPos.x * n.x, lightPos.x * n.y, lightPos.x * n.z, lightPos.x * planeD,
        lightPos.y * n.x, t + lightPos.y * n.y, lightPos.y * n.z, lightPos.y * planeD,
        lightPos.z * n.x, lightPos.z * n.y, t + lightPos.z * n.z, lightPos.z * planeD,
        n.x, n.y, n.z, dotLN
    );
}

Matrix4x4 Matrix4x4::worldToLocal(const Vector3& position, const Quaternion& rotation, const Vector3& scale) {
    Matrix4x4 world = fromTRS(position, rotation, scale);
    return world.inverse();
}

Matrix4x4 Matrix4x4::localToWorld(const Vector3& position, const Quaternion& rotation, const Vector3& scale) {
    return fromTRS(position, rotation, scale);
}

// 调试输出
std::ostream& operator<<(std::ostream& os, const Matrix4x4& mat) {
    for (int i = 0; i < 4; ++i) {
        os << "[";
        for (int j = 0; j < 4; ++j) {
            os << mat.m[i][j];
            if (j < 3) os << ", ";
        }
        os << "]\n";
    }
    return os;
}

} // namespace AstraAlgebra
