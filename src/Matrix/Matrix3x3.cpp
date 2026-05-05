// Matrix3x3.cpp - 3x3矩阵类实现
// 旋转、2D仿射变换那些

#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

#ifdef __SSE2__
#include <emmintrin.h>
#elif defined(_MSC_VER)
#include <xmmintrin.h>
#endif

namespace AstraAlgebra {

// 初始化静态常量
const Matrix3x3 Matrix3x3::identity = Matrix3x3(1.0f, 0.0f, 0.0f, 0.0f, 1.0f, 0.0f, 0.0f, 0.0f, 1.0f);
const Matrix3x3 Matrix3x3::zero = Matrix3x3(0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f);

// 赋值操作符
Matrix3x3& Matrix3x3::operator=(const Matrix3x3& other) {
    if (this != &other) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                m[i][j] = other.m[i][j];
    }
    return *this;
}

Matrix3x3& Matrix3x3::operator=(Matrix3x3&& other) noexcept {
    if (this != &other) {
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                m[i][j] = other.m[i][j];
                other.m[i][j] = 0.0f;
            }
    }
    return *this;
}

Matrix3x3& Matrix3x3::operator+=(const Matrix3x3& other) {
    __m128 row0 = _mm_set_ps(0.0f, m[0][2], m[0][1], m[0][0]);
    __m128 row1 = _mm_set_ps(0.0f, m[1][2], m[1][1], m[1][0]);
    __m128 row2 = _mm_set_ps(0.0f, m[2][2], m[2][1], m[2][0]);
    __m128 otherRow0 = _mm_set_ps(0.0f, other.m[0][2], other.m[0][1], other.m[0][0]);
    __m128 otherRow1 = _mm_set_ps(0.0f, other.m[1][2], other.m[1][1], other.m[1][0]);
    __m128 otherRow2 = _mm_set_ps(0.0f, other.m[2][2], other.m[2][1], other.m[2][0]);
    __m128 res0 = _mm_add_ps(row0, otherRow0);
    __m128 res1 = _mm_add_ps(row1, otherRow1);
    __m128 res2 = _mm_add_ps(row2, otherRow2);
    _mm_store_ss(&m[0][0], res0);
    _mm_store_ss(&m[0][1], _mm_shuffle_ps(res0, res0, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[0][2], _mm_shuffle_ps(res0, res0, _MM_SHUFFLE(0, 3, 1, 2)));
    _mm_store_ss(&m[1][0], res1);
    _mm_store_ss(&m[1][1], _mm_shuffle_ps(res1, res1, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[1][2], _mm_shuffle_ps(res1, res1, _MM_SHUFFLE(0, 3, 1, 2)));
    _mm_store_ss(&m[2][0], res2);
    _mm_store_ss(&m[2][1], _mm_shuffle_ps(res2, res2, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[2][2], _mm_shuffle_ps(res2, res2, _MM_SHUFFLE(0, 3, 1, 2)));
    return *this;
}

Matrix3x3& Matrix3x3::operator-=(const Matrix3x3& other) {
    __m128 row0 = _mm_set_ps(0.0f, m[0][2], m[0][1], m[0][0]);
    __m128 row1 = _mm_set_ps(0.0f, m[1][2], m[1][1], m[1][0]);
    __m128 row2 = _mm_set_ps(0.0f, m[2][2], m[2][1], m[2][0]);
    __m128 otherRow0 = _mm_set_ps(0.0f, other.m[0][2], other.m[0][1], other.m[0][0]);
    __m128 otherRow1 = _mm_set_ps(0.0f, other.m[1][2], other.m[1][1], other.m[1][0]);
    __m128 otherRow2 = _mm_set_ps(0.0f, other.m[2][2], other.m[2][1], other.m[2][0]);
    __m128 res0 = _mm_sub_ps(row0, otherRow0);
    __m128 res1 = _mm_sub_ps(row1, otherRow1);
    __m128 res2 = _mm_sub_ps(row2, otherRow2);
    _mm_store_ss(&m[0][0], res0);
    _mm_store_ss(&m[0][1], _mm_shuffle_ps(res0, res0, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[0][2], _mm_shuffle_ps(res0, res0, _MM_SHUFFLE(0, 3, 1, 2)));
    _mm_store_ss(&m[1][0], res1);
    _mm_store_ss(&m[1][1], _mm_shuffle_ps(res1, res1, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[1][2], _mm_shuffle_ps(res1, res1, _MM_SHUFFLE(0, 3, 1, 2)));
    _mm_store_ss(&m[2][0], res2);
    _mm_store_ss(&m[2][1], _mm_shuffle_ps(res2, res2, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[2][2], _mm_shuffle_ps(res2, res2, _MM_SHUFFLE(0, 3, 1, 2)));
    return *this;
}

Matrix3x3& Matrix3x3::operator*=(const Matrix3x3& other) {
    Matrix3x3 temp = *this * other;
    *this = temp;
    return *this;
}

Matrix3x3& Matrix3x3::operator*=(float scalar) {
    __m128 scalarVec = _mm_set1_ps(scalar);
    __m128 row0 = _mm_set_ps(0.0f, m[0][2], m[0][1], m[0][0]);
    __m128 row1 = _mm_set_ps(0.0f, m[1][2], m[1][1], m[1][0]);
    __m128 row2 = _mm_set_ps(0.0f, m[2][2], m[2][1], m[2][0]);
    __m128 res0 = _mm_mul_ps(row0, scalarVec);
    __m128 res1 = _mm_mul_ps(row1, scalarVec);
    __m128 res2 = _mm_mul_ps(row2, scalarVec);
    _mm_store_ss(&m[0][0], res0);
    _mm_store_ss(&m[0][1], _mm_shuffle_ps(res0, res0, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[0][2], _mm_shuffle_ps(res0, res0, _MM_SHUFFLE(0, 3, 1, 2)));
    _mm_store_ss(&m[1][0], res1);
    _mm_store_ss(&m[1][1], _mm_shuffle_ps(res1, res1, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[1][2], _mm_shuffle_ps(res1, res1, _MM_SHUFFLE(0, 3, 1, 2)));
    _mm_store_ss(&m[2][0], res2);
    _mm_store_ss(&m[2][1], _mm_shuffle_ps(res2, res2, _MM_SHUFFLE(0, 3, 2, 1)));
    _mm_store_ss(&m[2][2], _mm_shuffle_ps(res2, res2, _MM_SHUFFLE(0, 3, 1, 2)));
    return *this;
}

Matrix3x3& Matrix3x3::operator/=(float scalar) {
    float invScalar = 1.0f / scalar;
    return *this *= invScalar;
}

Matrix3x3& Matrix3x3::transpose() {
    for (int i = 0; i < 3; ++i)
        for (int j = i + 1; j < 3; ++j) {
            float temp = m[i][j];
            m[i][j] = m[j][i];
            m[j][i] = temp;
        }
    return *this;
}

Matrix3x3 Matrix3x3::inverse() const {
    float det = determinant();
    if (Math::approximatelyEqual(det, 0.0f)) {
        return Matrix3x3::zero;
    }
    float invDet = 1.0f / det;
    Matrix3x3 result;
    result.m[0][0] = (m[1][1]*m[2][2]-m[1][2]*m[2][1])*invDet;
    result.m[0][1] = (m[0][2]*m[2][1]-m[0][1]*m[2][2])*invDet;
    result.m[0][2] = (m[0][1]*m[1][2]-m[0][2]*m[1][1])*invDet;
    result.m[1][0] = (m[1][2]*m[2][0]-m[1][0]*m[2][2])*invDet;
    result.m[1][1] = (m[0][0]*m[2][2]-m[0][2]*m[2][0])*invDet;
    result.m[1][2] = (m[0][2]*m[1][0]-m[0][0]*m[1][2])*invDet;
    result.m[2][0] = (m[1][0]*m[2][1]-m[1][1]*m[2][0])*invDet;
    result.m[2][1] = (m[0][1]*m[2][0]-m[0][0]*m[2][1])*invDet;
    result.m[2][2] = (m[0][0]*m[1][1]-m[0][1]*m[1][0])*invDet;
    return result;
}

Matrix3x3& Matrix3x3::invert() {
    *this = inverse();
    return *this;
}

Matrix3x3 Matrix3x3::rotationX(float angle, bool isDegrees) {
    if (isDegrees) angle = Math::degToRad(angle);
    float c = std::cos(angle), s = std::sin(angle);
    return Matrix3x3(1.0f,0.0f,0.0f, 0.0f,c,-s, 0.0f,s,c);
}

Matrix3x3 Matrix3x3::rotationY(float angle, bool isDegrees) {
    if (isDegrees) angle = Math::degToRad(angle);
    float c = std::cos(angle), s = std::sin(angle);
    return Matrix3x3(c,0.0f,s, 0.0f,1.0f,0.0f, -s,0.0f,c);
}

Matrix3x3 Matrix3x3::rotationZ(float angle, bool isDegrees) {
    if (isDegrees) angle = Math::degToRad(angle);
    float c = std::cos(angle), s = std::sin(angle);
    return Matrix3x3(c,-s,0.0f, s,c,0.0f, 0.0f,0.0f,1.0f);
}

Matrix3x3 Matrix3x3::rotation(const Vector3& axis, float angle, bool isDegrees) {
    if (isDegrees) angle = Math::degToRad(angle);
    Vector3 n = axis.normalized();
    float x = n.x, y = n.y, z = n.z;
    float c = std::cos(angle), s = std::sin(angle), omc = 1.0f - c;
    return Matrix3x3(
        c+x*x*omc, x*y*omc-z*s, x*z*omc+y*s,
        y*x*omc+z*s, c+y*y*omc, y*z*omc-x*s,
        z*x*omc-y*s, z*y*omc+x*s, c+z*z*omc
    );
}

std::ostream& operator<<(std::ostream& os, const Matrix3x3& mat) {
    os << "Matrix3x3(\n";
    os << "  [ " << mat.m[0][0] << ", " << mat.m[0][1] << ", " << mat.m[0][2] << " ]\n";
    os << "  [ " << mat.m[1][0] << ", " << mat.m[1][1] << ", " << mat.m[1][2] << " ]\n";
    os << "  [ " << mat.m[2][0] << ", " << mat.m[2][1] << ", " << mat.m[2][2] << " ]\n";
    os << ")";
    return os;
}

// 幂法计算最大特征值和对应特征向量
float Matrix3x3::powerMethod(Vector3& eigenvector, float tolerance, int maxIterations) const {
    if (eigenvector.lengthSquared() < Math::EPSILON) {
        eigenvector = Vector3(1.0f, 0.0f, 0.0f);
    }
    eigenvector.normalize();
    float eigenvalue = 0.0f, prevEigenvalue = 0.0f;
    if (maxIterations <= 0) maxIterations = 1000;
    if (tolerance <= 0.0f) tolerance = Math::EPSILON * 10.0f;
    for (int iter = 0; iter < maxIterations; ++iter) {
        Vector3 temp = *this * eigenvector;
        eigenvalue = eigenvector.dot(temp);
        float norm = temp.length();
        if (norm < Math::EPSILON) break;
        temp /= norm;
        eigenvector = temp;
        if (std::abs(eigenvalue - prevEigenvalue) < tolerance) break;
        prevEigenvalue = eigenvalue;
    }
    return eigenvalue;
}

// 计算3x3矩阵的所有特征值
float Matrix3x3::eigenvalues(float* eigenvalues) const {
    float trace = this->trace();
    float det = determinant();
    float p1 = m[0][1]*m[1][0] + m[0][2]*m[2][0] + m[1][2]*m[2][1];
    float p2 = trace*trace - 3.0f*p1;
    if (p2 < Math::EPSILON) {
        eigenvalues[0] = eigenvalues[1] = eigenvalues[2] = trace / 3.0f;
        return 1;
    }
    float q = (2.0f*trace*trace*trace - 9.0f*trace*p1 + 27.0f*det) / 27.0f;
    float qOver2 = q / 2.0f;
    float discriminant = qOver2*qOver2 - p2*p2*p2 / 27.0f;
    if (discriminant > 0.0f) {
        float sqrtDisc = std::sqrt(discriminant);
        float u = -qOver2 + sqrtDisc;
        float v = -qOver2 - sqrtDisc;
        u = (u >= 0) ? std::pow(u, 1.0f/3.0f) : -std::pow(-u, 1.0f/3.0f);
        v = (v >= 0) ? std::pow(v, 1.0f/3.0f) : -std::pow(-v, 1.0f/3.0f);
        eigenvalues[0] = (trace + u + v) / 3.0f;
        eigenvalues[1] = eigenvalues[2] = 0.0f;
        return 1;
    } else {
        float r = std::sqrt(p2 / 3.0f);
        float theta = std::acos(qOver2 / (r*r*r)) / 3.0f;
        eigenvalues[0] = trace / 3.0f + 2.0f*r*std::cos(theta);
        eigenvalues[1] = trace / 3.0f + 2.0f*r*std::cos(theta - 2.0f*Math::PI/3.0f);
        eigenvalues[2] = trace / 3.0f + 2.0f*r*std::cos(theta - 4.0f*Math::PI/3.0f);
        return 3;
    }
}

// 计算特征值和特征向量
float Matrix3x3::eigenvectors(float* eigenvalues, Vector3* eigenvectors) const {
    float numEigenvalues = this->eigenvalues(eigenvalues);
    for (int i = 0; i < static_cast<int>(numEigenvalues); ++i) {
        Matrix3x3 A = *this;
        A.m[0][0] -= eigenvalues[i];
        A.m[1][1] -= eigenvalues[i];
        A.m[2][2] -= eigenvalues[i];
        Vector3 v1(A.m[0][1]*A.m[1][2]-A.m[0][2]*A.m[1][1],
                   A.m[0][2]*A.m[1][0]-A.m[0][0]*A.m[1][2],
                   A.m[0][0]*A.m[1][1]-A.m[0][1]*A.m[1][0]);
        Vector3 v2(A.m[0][1]*A.m[2][2]-A.m[0][2]*A.m[2][1],
                   A.m[0][2]*A.m[2][0]-A.m[0][0]*A.m[2][2],
                   A.m[0][0]*A.m[2][1]-A.m[0][1]*A.m[2][0]);
        Vector3 v3(A.m[1][1]*A.m[2][2]-A.m[1][2]*A.m[2][1],
                   A.m[1][2]*A.m[2][0]-A.m[1][0]*A.m[2][2],
                   A.m[1][0]*A.m[2][1]-A.m[1][1]*A.m[2][0]);
        if (v1.lengthSquared() >= v2.lengthSquared() && v1.lengthSquared() >= v3.lengthSquared()) {
            eigenvectors[i] = v1;
        } else if (v2.lengthSquared() >= v3.lengthSquared()) {
            eigenvectors[i] = v2;
        } else {
            eigenvectors[i] = v3;
        }
        if (eigenvectors[i].lengthSquared() > Math::EPSILON) {
            eigenvectors[i].normalize();
        } else {
            eigenvectors[i] = Vector3(1.0f, 0.0f, 0.0f);
        }
    }
    return numEigenvalues;
}

} // namespace AstraAlgebra
