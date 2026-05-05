// Matrix2x2.cpp - 2x2矩阵类实现
// 2D变换用的，简单好用

#include <AstraAlgebra/Matrix/Matrix2x2.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>
#include <emmintrin.h>

namespace AstraAlgebra {

// 初始化静态常量
const Matrix2x2 Matrix2x2::identity = Matrix2x2(1.0f, 0.0f, 0.0f, 1.0f);
const Matrix2x2 Matrix2x2::zero = Matrix2x2(0.0f, 0.0f, 0.0f, 0.0f);

// 赋值操作符
Matrix2x2& Matrix2x2::operator=(const Matrix2x2& other) {
    if (this != &other) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                m[i][j] = other.m[i][j];
            }
        }
    }
    return *this;
}

Matrix2x2& Matrix2x2::operator=(Matrix2x2&& other) noexcept {
    if (this != &other) {
        for (int i = 0; i < 2; ++i) {
            for (int j = 0; j < 2; ++j) {
                m[i][j] = other.m[i][j];
                other.m[i][j] = 0.0f;
            }
        }
    }
    return *this;
}

Matrix2x2& Matrix2x2::operator+=(const Matrix2x2& other) {
    __m128 thisVec = _mm_set_ps(m[1][1], m[1][0], m[0][1], m[0][0]);
    __m128 otherVec = _mm_set_ps(other.m[1][1], other.m[1][0], other.m[0][1], other.m[0][0]);
    __m128 resultVec = _mm_add_ps(thisVec, otherVec);
    float temp[4];
    _mm_storeu_ps(temp, resultVec);
    m[0][0] = temp[0]; m[0][1] = temp[1];
    m[1][0] = temp[2]; m[1][1] = temp[3];
    return *this;
}

Matrix2x2& Matrix2x2::operator-=(const Matrix2x2& other) {
    __m128 thisVec = _mm_set_ps(m[1][1], m[1][0], m[0][1], m[0][0]);
    __m128 otherVec = _mm_set_ps(other.m[1][1], other.m[1][0], other.m[0][1], other.m[0][0]);
    __m128 resultVec = _mm_sub_ps(thisVec, otherVec);
    float temp[4];
    _mm_storeu_ps(temp, resultVec);
    m[0][0] = temp[0]; m[0][1] = temp[1];
    m[1][0] = temp[2]; m[1][1] = temp[3];
    return *this;
}

Matrix2x2& Matrix2x2::operator*=(const Matrix2x2& other) {
    Matrix2x2 temp = *this * other;
    *this = temp;
    return *this;
}

Matrix2x2& Matrix2x2::operator*=(float scalar) {
    __m128 thisVec = _mm_set_ps(m[1][1], m[1][0], m[0][1], m[0][0]);
    __m128 scalarVec = _mm_set1_ps(scalar);
    __m128 resultVec = _mm_mul_ps(thisVec, scalarVec);
    float temp[4];
    _mm_storeu_ps(temp, resultVec);
    m[0][0] = temp[0]; m[0][1] = temp[1];
    m[1][0] = temp[2]; m[1][1] = temp[3];
    return *this;
}

Matrix2x2& Matrix2x2::operator/=(float scalar) {
    float invScalar = 1.0f / scalar;
    return *this *= invScalar;
}

Matrix2x2& Matrix2x2::transpose() {
    float temp = m[0][1];
    m[0][1] = m[1][0];
    m[1][0] = temp;
    return *this;
}

Matrix2x2 Matrix2x2::inverse() const {
    float det = determinant();
    if (Math::approximatelyEqual(det, 0.0f)) {
        return Matrix2x2::zero;
    }
    float invDet = 1.0f / det;
    return Matrix2x2(
        m[1][1] * invDet, -m[0][1] * invDet,
        -m[1][0] * invDet, m[0][0] * invDet
    );
}

Matrix2x2& Matrix2x2::invert() {
    *this = inverse();
    return *this;
}

Matrix2x2 Matrix2x2::rotation(float angle, bool isDegrees) {
    if (isDegrees) {
        angle = Math::degToRad(angle);
    }
    float cosAngle = std::cos(angle);
    float sinAngle = std::sin(angle);
    return Matrix2x2(
        cosAngle, -sinAngle,
        sinAngle, cosAngle
    );
}

std::ostream& operator<<(std::ostream& os, const Matrix2x2& mat) {
    os << "Matrix2x2(\n";
    os << "  [ " << mat.m[0][0] << ", " << mat.m[0][1] << " ]\n";
    os << "  [ " << mat.m[1][0] << ", " << mat.m[1][1] << " ]\n";
    os << ")";
    return os;
}

} // namespace AstraAlgebra
