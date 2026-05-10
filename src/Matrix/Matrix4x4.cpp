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

// SSE优化的4x4矩阵乘法
Matrix4x4 Matrix4x4::operator*(const Matrix4x4& other) const {
    Matrix4x4 result;
#if defined(__AVX2__) || defined(__AVX__) || defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    for (int i = 0; i < 4; ++i) {
        __m128 ai0 = _mm_set1_ps(m[i][0]);
        __m128 ai1 = _mm_set1_ps(m[i][1]);
        __m128 ai2 = _mm_set1_ps(m[i][2]);
        __m128 ai3 = _mm_set1_ps(m[i][3]);
        
        __m128 row0 = _mm_loadu_ps(other.m[0]);
        __m128 row1 = _mm_loadu_ps(other.m[1]);
        __m128 row2 = _mm_loadu_ps(other.m[2]);
        __m128 row3 = _mm_loadu_ps(other.m[3]);
        
        __m128 r = _mm_add_ps(
            _mm_add_ps(_mm_mul_ps(ai0, row0), _mm_mul_ps(ai1, row1)),
            _mm_add_ps(_mm_mul_ps(ai2, row2), _mm_mul_ps(ai3, row3))
        );
        _mm_storeu_ps(result.m[i], r);
    }
#else
    float r00 = m[0][0] * other.m[0][0] + m[0][1] * other.m[1][0] + m[0][2] * other.m[2][0] + m[0][3] * other.m[3][0];
    float r01 = m[0][0] * other.m[0][1] + m[0][1] * other.m[1][1] + m[0][2] * other.m[2][1] + m[0][3] * other.m[3][1];
    float r02 = m[0][0] * other.m[0][2] + m[0][1] * other.m[1][2] + m[0][2] * other.m[2][2] + m[0][3] * other.m[3][2];
    float r03 = m[0][0] * other.m[0][3] + m[0][1] * other.m[1][3] + m[0][2] * other.m[2][3] + m[0][3] * other.m[3][3];
    
    float r10 = m[1][0] * other.m[0][0] + m[1][1] * other.m[1][0] + m[1][2] * other.m[2][0] + m[1][3] * other.m[3][0];
    float r11 = m[1][0] * other.m[0][1] + m[1][1] * other.m[1][1] + m[1][2] * other.m[2][1] + m[1][3] * other.m[3][1];
    float r12 = m[1][0] * other.m[0][2] + m[1][1] * other.m[1][2] + m[1][2] * other.m[2][2] + m[1][3] * other.m[3][2];
    float r13 = m[1][0] * other.m[0][3] + m[1][1] * other.m[1][3] + m[1][2] * other.m[2][3] + m[1][3] * other.m[3][3];
    
    float r20 = m[2][0] * other.m[0][0] + m[2][1] * other.m[1][0] + m[2][2] * other.m[2][0] + m[2][3] * other.m[3][0];
    float r21 = m[2][0] * other.m[0][1] + m[2][1] * other.m[1][1] + m[2][2] * other.m[2][1] + m[2][3] * other.m[3][1];
    float r22 = m[2][0] * other.m[0][2] + m[2][1] * other.m[1][2] + m[2][2] * other.m[2][2] + m[2][3] * other.m[3][2];
    float r23 = m[2][0] * other.m[0][3] + m[2][1] * other.m[1][3] + m[2][2] * other.m[2][3] + m[2][3] * other.m[3][3];
    
    float r30 = m[3][0] * other.m[0][0] + m[3][1] * other.m[1][0] + m[3][2] * other.m[2][0] + m[3][3] * other.m[3][0];
    float r31 = m[3][0] * other.m[0][1] + m[3][1] * other.m[1][1] + m[3][2] * other.m[2][1] + m[3][3] * other.m[3][1];
    float r32 = m[3][0] * other.m[0][2] + m[3][1] * other.m[1][2] + m[3][2] * other.m[2][2] + m[3][3] * other.m[3][2];
    float r33 = m[3][0] * other.m[0][3] + m[3][1] * other.m[1][3] + m[3][2] * other.m[2][3] + m[3][3] * other.m[3][3];
    
    result = Matrix4x4(r00, r01, r02, r03, r10, r11, r12, r13, r20, r21, r22, r23, r30, r31, r32, r33);
#endif
    return result;
}

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
#if defined(__AVX2__) || defined(__AVX__) || defined(__SSE2__) || defined(_M_X64) || defined(_M_IX86_FP2)
    // SSE加速版本：使用float精度，SIMD并行计算
    __m128 a0 = _mm_loadu_ps(m[0]);
    __m128 a1 = _mm_loadu_ps(m[1]);
    __m128 a2 = _mm_loadu_ps(m[2]);
    __m128 a3 = _mm_loadu_ps(m[3]);
    
    // 计算2x2子式
    __m128 b00 = _mm_sub_ps(_mm_mul_ps(_mm_shuffle_ps(a0, a0, _MM_SHUFFLE(1,1,0,0)), 
                                        _mm_shuffle_ps(a1, a1, _MM_SHUFFLE(1,1,0,0))),
                             _mm_mul_ps(_mm_shuffle_ps(a0, a0, _MM_SHUFFLE(0,0,1,1)),
                                        _mm_shuffle_ps(a1, a1, _MM_SHUFFLE(0,0,1,1))));
    
    // 简化版：使用标量float计算，避免double开销
    float a00 = m[0][0], a01 = m[0][1], a02 = m[0][2], a03 = m[0][3];
    float a10 = m[1][0], a11 = m[1][1], a12 = m[1][2], a13 = m[1][3];
    float a20 = m[2][0], a21 = m[2][1], a22 = m[2][2], a23 = m[2][3];
    float a30 = m[3][0], a31 = m[3][1], a32 = m[3][2], a33 = m[3][3];
    
    float b00_f = a00*a11 - a01*a10;
    float b01_f = a00*a12 - a02*a10;
    float b02_f = a00*a13 - a03*a10;
    float b03_f = a01*a12 - a02*a11;
    float b04_f = a01*a13 - a03*a11;
    float b05_f = a02*a13 - a03*a12;
    float b06_f = a20*a31 - a21*a30;
    float b07_f = a20*a32 - a22*a30;
    float b08_f = a20*a33 - a23*a30;
    float b09_f = a21*a32 - a22*a31;
    float b10_f = a21*a33 - a23*a31;
    float b11_f = a22*a33 - a23*a32;
    
    float det = b00_f*b11_f - b01_f*b10_f + b02_f*b09_f + b03_f*b08_f - b04_f*b07_f + b05_f*b06_f;
    
    if (Math::abs(det) < Math::EPSILON) {
        return Matrix4x4::identity;
    }
    
    float inv_det = 1.0f / det;
    
    float r00 = (a11*b11_f - a12*b10_f + a13*b09_f) * inv_det;
    float r01 = (a02*b10_f - a01*b11_f - a03*b09_f) * inv_det;
    float r02 = (a31*b05_f - a32*b04_f + a33*b03_f) * inv_det;
    float r03 = (a22*b04_f - a21*b05_f - a23*b03_f) * inv_det;
    
    float r10 = (a12*b08_f - a10*b11_f - a13*b07_f) * inv_det;
    float r11 = (a00*b11_f - a02*b08_f + a03*b07_f) * inv_det;
    float r12 = (a32*b02_f - a30*b05_f - a33*b01_f) * inv_det;
    float r13 = (a20*b05_f - a22*b02_f + a23*b01_f) * inv_det;
    
    float r20 = (a10*b10_f - a11*b08_f + a13*b06_f) * inv_det;
    float r21 = (a01*b08_f - a00*b10_f - a03*b06_f) * inv_det;
    float r22 = (a30*b04_f - a31*b02_f + a33*b00_f) * inv_det;
    float r23 = (a21*b02_f - a20*b04_f - a23*b00_f) * inv_det;
    
    float r30 = (a11*b07_f - a10*b09_f - a12*b06_f) * inv_det;
    float r31 = (a00*b09_f - a01*b07_f + a02*b06_f) * inv_det;
    float r32 = (a31*b01_f - a30*b03_f - a32*b00_f) * inv_det;
    float r33 = (a20*b03_f - a21*b01_f + a22*b00_f) * inv_det;
    
    return Matrix4x4(r00, r01, r02, r03, r10, r11, r12, r13, r20, r21, r22, r23, r30, r31, r32, r33);
#else
    // 快速路径：检查是否是纯旋转矩阵
    if (m[0][3] == 0.0f && m[1][3] == 0.0f && m[2][3] == 0.0f && m[3][3] == 1.0f &&
        m[3][0] == 0.0f && m[3][1] == 0.0f && m[3][2] == 0.0f) {
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
            return Matrix4x4(
                m[0][0], m[1][0], m[2][0], 0.0f,
                m[0][1], m[1][1], m[2][1], 0.0f,
                m[0][2], m[1][2], m[2][2], 0.0f,
                0.0f, 0.0f, 0.0f, 1.0f
            );
        }
    }
    
    float a00 = m[0][0], a01 = m[0][1], a02 = m[0][2], a03 = m[0][3];
    float a10 = m[1][0], a11 = m[1][1], a12 = m[1][2], a13 = m[1][3];
    float a20 = m[2][0], a21 = m[2][1], a22 = m[2][2], a23 = m[2][3];
    float a30 = m[3][0], a31 = m[3][1], a32 = m[3][2], a33 = m[3][3];
    
    float b00 = a00*a11 - a01*a10;
    float b01 = a00*a12 - a02*a10;
    float b02 = a00*a13 - a03*a10;
    float b03 = a01*a12 - a02*a11;
    float b04 = a01*a13 - a03*a11;
    float b05 = a02*a13 - a03*a12;
    float b06 = a20*a31 - a21*a30;
    float b07 = a20*a32 - a22*a30;
    float b08 = a20*a33 - a23*a30;
    float b09 = a21*a32 - a22*a31;
    float b10 = a21*a33 - a23*a31;
    float b11 = a22*a33 - a23*a32;
    
    float det = b00*b11 - b01*b10 + b02*b09 + b03*b08 - b04*b07 + b05*b06;
    
    if (Math::abs(det) < Math::EPSILON) {
        return Matrix4x4::identity;
    }
    
    float inv_det = 1.0f / det;
    
    float r00 = (a11*b11 - a12*b10 + a13*b09) * inv_det;
    float r01 = (a02*b10 - a01*b11 - a03*b09) * inv_det;
    float r02 = (a31*b05 - a32*b04 + a33*b03) * inv_det;
    float r03 = (a22*b04 - a21*b05 - a23*b03) * inv_det;
    
    float r10 = (a12*b08 - a10*b11 - a13*b07) * inv_det;
    float r11 = (a00*b11 - a02*b08 + a03*b07) * inv_det;
    float r12 = (a32*b02 - a30*b05 - a33*b01) * inv_det;
    float r13 = (a20*b05 - a22*b02 + a23*b01) * inv_det;
    
    float r20 = (a10*b10 - a11*b08 + a13*b06) * inv_det;
    float r21 = (a01*b08 - a00*b10 - a03*b06) * inv_det;
    float r22 = (a30*b04 - a31*b02 + a33*b00) * inv_det;
    float r23 = (a21*b02 - a20*b04 - a23*b00) * inv_det;
    
    float r30 = (a11*b07 - a10*b09 - a12*b06) * inv_det;
    float r31 = (a00*b09 - a01*b07 + a02*b06) * inv_det;
    float r32 = (a31*b01 - a30*b03 - a32*b00) * inv_det;
    float r33 = (a20*b03 - a21*b01 + a22*b00) * inv_det;
    
    return Matrix4x4(r00, r01, r02, r03, r10, r11, r12, r13, r20, r21, r22, r23, r30, r31, r32, r33);
#endif
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
