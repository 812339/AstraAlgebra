// Matrix4x4d.cpp - 双精度4x4矩阵类实现
// 精度要求高的场景用，比如CAD、科学计算

#include <AstraAlgebra/Matrix/Matrix4x4d.h>
#include <cmath>

namespace AstraAlgebra {

double Matrix4x4d::determinant() const {
    double det = 0.0;
    for (int i = 0; i < 4; ++i) {
        double minor[3][3];
        int mi = 0;
        for (int j = 0; j < 4; ++j) {
            if (j == i) continue;
            int mj = 0;
            for (int k = 1; k < 4; ++k) {
                minor[mi][mj++] = m[j][k];
            }
            ++mi;
        }
        double minorDet = minor[0][0] * (minor[1][1] * minor[2][2] - minor[1][2] * minor[2][1])
                        - minor[0][1] * (minor[1][0] * minor[2][2] - minor[1][2] * minor[2][0])
                        + minor[0][2] * (minor[1][0] * minor[2][1] - minor[1][1] * minor[2][0]);
        det += (i % 2 == 0 ? 1 : -1) * m[i][0] * minorDet;
    }
    return det;
}

Matrix4x4d Matrix4x4d::inverse() const {
    double det = determinant();
    if (Math::abs(det) < Math::EPSILON_D) return identity;
    
    double invDet = 1.0 / det;
    Matrix4x4d result;
    
    for (int i = 0; i < 4; ++i) {
        for (int j = 0; j < 4; ++j) {
            double minor[3][3];
            int mi = 0;
            for (int k = 0; k < 4; ++k) {
                if (k == i) continue;
                int mj = 0;
                for (int l = 0; l < 4; ++l) {
                    if (l == j) continue;
                    minor[mi][mj++] = m[k][l];
                }
                ++mi;
            }
            double cofactor = minor[0][0] * (minor[1][1] * minor[2][2] - minor[1][2] * minor[2][1])
                            - minor[0][1] * (minor[1][0] * minor[2][2] - minor[1][2] * minor[2][0])
                            + minor[0][2] * (minor[1][0] * minor[2][1] - minor[1][1] * minor[2][0]);
            result.m[j][i] = ((i + j) % 2 == 0 ? 1 : -1) * cofactor * invDet;
        }
    }
    
    return result;
}

Matrix4x4d Matrix4x4d::lookAt(const Vector3d& eye, const Vector3d& target, const Vector3d& up) {
    Vector3d f = (target - eye).normalized();
    Vector3d r = f.cross(up).normalized();
    Vector3d u = r.cross(f);
    
    Matrix4x4d result;
    result.m[0][0] = r.x; result.m[0][1] = r.y; result.m[0][2] = r.z; result.m[0][3] = -r.dot(eye);
    result.m[1][0] = u.x; result.m[1][1] = u.y; result.m[1][2] = u.z; result.m[1][3] = -u.dot(eye);
    result.m[2][0] = -f.x; result.m[2][1] = -f.y; result.m[2][2] = -f.z; result.m[2][3] = f.dot(eye);
    result.m[3][0] = 0.0; result.m[3][1] = 0.0; result.m[3][2] = 0.0; result.m[3][3] = 1.0;
    
    return result;
}

Matrix4x4d Matrix4x4d::perspective(double fov, double aspect, double near, double far) {
    double tanHalfFov = std::tan(fov / 2.0);
    Matrix4x4d result = zeroMatrix();
    
    result.m[0][0] = 1.0 / (aspect * tanHalfFov);
    result.m[1][1] = 1.0 / tanHalfFov;
    result.m[2][2] = -(far + near) / (far - near);
    result.m[2][3] = -(2.0 * far * near) / (far - near);
    result.m[3][2] = -1.0;
    
    return result;
}

Matrix4x4d Matrix4x4d::orthographic(double left, double right, double bottom, double top, double near, double far) {
    Matrix4x4d result = zeroMatrix();
    
    result.m[0][0] = 2.0 / (right - left);
    result.m[1][1] = 2.0 / (top - bottom);
    result.m[2][2] = -2.0 / (far - near);
    result.m[0][3] = -(right + left) / (right - left);
    result.m[1][3] = -(top + bottom) / (top - bottom);
    result.m[2][3] = -(far + near) / (far - near);
    result.m[3][3] = 1.0;
    
    return result;
}

} // namespace AstraAlgebra
