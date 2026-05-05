// MatrixDecomposition - 矩阵分解，LU/QR/Cholesky/SVD那些，高级线性代数用的

#pragma once

#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <vector>
#include <cmath>

namespace AstraAlgebra {

// 矩阵分解工具类
class MatrixDecomposition {
public:
    // LU分解：A = P * L * U
    // 返回: L(下三角), U(上三角), P(置换矩阵)
    static bool luDecompose(const Matrix4x4& A, Matrix4x4& L, Matrix4x4& U, Matrix4x4& P);
    
    // QR分解：A = Q * R
    // 返回: Q(正交矩阵), R(上三角矩阵)
    static bool qrDecompose(const Matrix4x4& A, Matrix4x4& Q, Matrix4x4& R);
    
    // Cholesky分解：A = L * L^T (仅适用于对称正定矩阵)
    static bool choleskyDecompose(const Matrix4x4& A, Matrix4x4& L);
    
    // 特征值分解（3x3对称矩阵）
    // 返回: 特征值(降序排列)和对应的特征向量
    static bool eigenDecompose3x3(const Matrix3x3& A, Vector3& eigenvalues, 
                                  Vector3& eigenvector1, Vector3& eigenvector2, Vector3& eigenvector3);
    
    // 奇异值分解(SVD)简化版：A = U * S * V^T
    // 适用于3x3矩阵，用于物理引擎和图形学
    static bool svdDecompose3x3(const Matrix3x3& A, Matrix3x3& U, Vector3& S, Matrix3x3& V);
    
    // 极分解：A = R * S (旋转 * 缩放/剪切)
    static bool polarDecompose(const Matrix4x4& A, Matrix4x4& R, Matrix4x4& S);
    
    // 从变换矩阵中提取平移、旋转、缩放
    static bool decomposeTransform(const Matrix4x4& transform, Vector3& translation, 
                                   Quaternion& rotation, Vector3& scale);
    
    // 求解线性方程组 Ax = b (使用LU分解)
    static bool solveLinearSystem(const Matrix4x4& A, const Vector3& b, Vector3& x);
    
    // 计算矩阵的秩
    static int computeRank(const Matrix4x4& A, float tolerance = 1e-6f);
    
    // 计算矩阵的条件数
    static float computeConditionNumber(const Matrix4x4& A);

private:
    // 辅助函数：3x3矩阵的特征值计算（使用Jacobi迭代）
    static bool jacobiEigenvalue3x3(const Matrix3x3& A, Vector3& eigenvalues, Matrix3x3& eigenvectors);
    
    // 辅助函数：Givens旋转
    static void givensRotation(float& a, float& b, float& c, float& s);
};

// ========== 内联实现 ==========

inline bool MatrixDecomposition::luDecompose(const Matrix4x4& A, Matrix4x4& L, Matrix4x4& U, Matrix4x4& P) {
    const int n = 4;
    float a[4][4];
    
    // 复制矩阵
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            a[i][j] = A.m[i][j];
    
    // 初始化P为单位矩阵
    P = Matrix4x4::identity;
    L = Matrix4x4::zero;
    U = Matrix4x4::zero;
    
    // 高斯消元法（带部分主元）
    for (int k = 0; k < n; ++k) {
        // 找主元
        int maxRow = k;
        float maxVal = std::abs(a[k][k]);
        for (int i = k + 1; i < n; ++i) {
            if (std::abs(a[i][k]) > maxVal) {
                maxVal = std::abs(a[i][k]);
                maxRow = i;
            }
        }
        
        if (maxVal < Math::EPSILON) return false; // 奇异矩阵
        
        // 交换行
        if (maxRow != k) {
            for (int j = 0; j < n; ++j) {
                std::swap(a[k][j], a[maxRow][j]);
                std::swap(P.m[k][j], P.m[maxRow][j]);
            }
        }
        
        // 消元
        for (int i = k + 1; i < n; ++i) {
            a[i][k] /= a[k][k];
            for (int j = k + 1; j < n; ++j) {
                a[i][j] -= a[i][k] * a[k][j];
            }
        }
    }
    
    // 提取L和U
    for (int i = 0; i < n; ++i) {
        L.m[i][i] = 1.0f;
        for (int j = 0; j < i; ++j) {
            L.m[i][j] = a[i][j];
        }
        for (int j = i; j < n; ++j) {
            U.m[i][j] = a[i][j];
        }
    }
    
    return true;
}

inline bool MatrixDecomposition::qrDecompose(const Matrix4x4& A, Matrix4x4& Q, Matrix4x4& R) {
    const int n = 4;
    float a[4][4];
    
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            a[i][j] = A.m[i][j];
    
    Q = Matrix4x4::identity;
    R = Matrix4x4::zero;
    
    // 使用Gram-Schmidt正交化
    for (int j = 0; j < n; ++j) {
        // 复制第j列
        float v[4];
        for (int i = 0; i < n; ++i) v[i] = a[i][j];
        
        // 减去前面列的投影
        for (int i = 0; i < j; ++i) {
            float dot = 0.0f;
            for (int k = 0; k < n; ++k) {
                dot += a[k][i] * v[k];
            }
            R.m[i][j] = dot;
            for (int k = 0; k < n; ++k) {
                v[k] -= dot * a[k][i];
            }
        }
        
        // 归一化
        float norm = 0.0f;
        for (int i = 0; i < n; ++i) norm += v[i] * v[i];
        norm = std::sqrt(norm);
        
        if (norm < Math::EPSILON) return false;
        
        R.m[j][j] = norm;
        for (int i = 0; i < n; ++i) {
            a[i][j] = v[i] / norm;
        }
    }
    
    // 提取Q
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j)
            Q.m[i][j] = a[i][j];
    
    return true;
}

inline bool MatrixDecomposition::choleskyDecompose(const Matrix4x4& A, Matrix4x4& L) {
    const int n = 4;
    L = Matrix4x4::zero;
    
    for (int i = 0; i < n; ++i) {
        for (int j = 0; j <= i; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < j; ++k) {
                sum += L.m[i][k] * L.m[j][k];
            }
            
            if (i == j) {
                float val = A.m[i][i] - sum;
                if (val <= 0.0f) return false; // 不是正定矩阵
                L.m[i][i] = std::sqrt(val);
            } else {
                if (std::abs(L.m[j][j]) < Math::EPSILON) return false;
                L.m[i][j] = (A.m[i][j] - sum) / L.m[j][j];
            }
        }
    }
    
    return true;
}

inline bool MatrixDecomposition::jacobiEigenvalue3x3(const Matrix3x3& A, Vector3& eigenvalues, Matrix3x3& eigenvectors) {
    const int maxIterations = 100;
    const float tolerance = 1e-8f;
    
    float a[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j)
            a[i][j] = A.m[i][j];
    
    eigenvectors = Matrix3x3::identity;
    
    for (int iter = 0; iter < maxIterations; ++iter) {
        // 找最大非对角元素
        int p = 0, q = 1;
        float maxVal = std::abs(a[0][1]);
        
        for (int i = 0; i < 3; ++i) {
            for (int j = i + 1; j < 3; ++j) {
                if (std::abs(a[i][j]) > maxVal) {
                    maxVal = std::abs(a[i][j]);
                    p = i; q = j;
                }
            }
        }
        
        if (maxVal < tolerance) break;
        
        // 计算旋转角度
        float theta;
        if (std::abs(a[p][p] - a[q][q]) < Math::EPSILON) {
            theta = (a[p][q] > 0.0f) ? Math::PI_4 : -Math::PI_4;
        } else {
            theta = 0.5f * std::atan2(2.0f * a[p][q], a[p][p] - a[q][q]);
        }
        
        float c = std::cos(theta);
        float s = std::sin(theta);
        
        // 应用旋转
        float app = c * c * a[p][p] - 2.0f * s * c * a[p][q] + s * s * a[q][q];
        float aqq = s * s * a[p][p] + 2.0f * s * c * a[p][q] + c * c * a[q][q];
        float apq = (c * c - s * s) * a[p][q] + s * c * (a[p][p] - a[q][q]);
        
        a[p][p] = app;
        a[q][q] = aqq;
        a[p][q] = apq;
        a[q][p] = apq;
        
        for (int i = 0; i < 3; ++i) {
            if (i != p && i != q) {
                float aip = c * a[i][p] - s * a[i][q];
                float aiq = s * a[i][p] + c * a[i][q];
                a[i][p] = aip;
                a[i][q] = aiq;
                a[p][i] = aip;
                a[q][i] = aiq;
            }
        }
        
        // 更新特征向量
        for (int i = 0; i < 3; ++i) {
            float vip = c * eigenvectors.m[i][p] - s * eigenvectors.m[i][q];
            float viq = s * eigenvectors.m[i][p] + c * eigenvectors.m[i][q];
            eigenvectors.m[i][p] = vip;
            eigenvectors.m[i][q] = viq;
        }
    }
    
    eigenvalues = Vector3(a[0][0], a[1][1], a[2][2]);
    return true;
}

inline bool MatrixDecomposition::eigenDecompose3x3(const Matrix3x3& A, Vector3& eigenvalues,
                                                    Vector3& eigenvector1, Vector3& eigenvector2, Vector3& eigenvector3) {
    Matrix3x3 eigenvectors;
    if (!jacobiEigenvalue3x3(A, eigenvalues, eigenvectors)) return false;
    
    eigenvector1 = Vector3(eigenvectors.m[0][0], eigenvectors.m[1][0], eigenvectors.m[2][0]);
    eigenvector2 = Vector3(eigenvectors.m[0][1], eigenvectors.m[1][1], eigenvectors.m[2][1]);
    eigenvector3 = Vector3(eigenvectors.m[0][2], eigenvectors.m[1][2], eigenvectors.m[2][2]);
    
    return true;
}

inline bool MatrixDecomposition::svdDecompose3x3(const Matrix3x3& A, Matrix3x3& U, Vector3& S, Matrix3x3& V) {
    // 简化SVD实现：A^T * A = V * S^2 * V^T
    Matrix3x3 AtA;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            AtA.m[i][j] = 0.0f;
            for (int k = 0; k < 3; ++k) {
                AtA.m[i][j] += A.m[k][i] * A.m[k][j];
            }
        }
    }
    
    // 特征分解得到V和S
    Matrix3x3 eigenvectors;
    if (!jacobiEigenvalue3x3(AtA, S, eigenvectors)) return false;
    
    // S是特征值，需要开方得到奇异值
    S.x = std::sqrt(std::abs(S.x));
    S.y = std::sqrt(std::abs(S.y));
    S.z = std::sqrt(std::abs(S.z));
    
    V = eigenvectors;
    
    // 计算U = A * V * S^-1
    U = Matrix3x3::zero;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            float sum = 0.0f;
            for (int k = 0; k < 3; ++k) {
                sum += A.m[i][k] * V.m[k][j];
            }
            if (S[j] > Math::EPSILON) {
                U.m[i][j] = sum / S[j];
            }
        }
    }
    
    return true;
}

inline bool MatrixDecomposition::polarDecompose(const Matrix4x4& A, Matrix4x4& R, Matrix4x4& S) {
    // 使用迭代法计算极分解
    // A = R * S, 其中R是正交矩阵，S是对称正定矩阵
    
    Matrix4x4 Ak = A;
    Matrix4x4 Ak_inv;
    
    // 迭代计算正交部分
    for (int i = 0; i < 10; ++i) {
        Ak_inv = Ak.inverse();
        if (std::abs(Ak_inv.determinant()) < Math::EPSILON) return false;
        
        Matrix4x4 Ak_T = Ak.transposed();
        float norm_diff = 0.0f;
        for (int r = 0; r < 4; ++r) {
            for (int c = 0; c < 4; ++c) {
                float diff = Ak_T.m[r][c] - Ak_inv.m[r][c];
                norm_diff += diff * diff;
            }
        }
        
        if (std::sqrt(norm_diff) < Math::EPSILON) break;
        
        Ak = (Ak_T + Ak_inv) * 0.5f;
    }
    
    R = Ak;
    S = R.transposed() * A;
    
    return true;
}

inline bool MatrixDecomposition::decomposeTransform(const Matrix4x4& transform, Vector3& translation,
                                                     Quaternion& rotation, Vector3& scale) {
    // 提取平移
    translation = Vector3(transform.m[0][3], transform.m[1][3], transform.m[2][3]);
    
    // 提取缩放
    Vector3 row0(transform.m[0][0], transform.m[0][1], transform.m[0][2]);
    Vector3 row1(transform.m[1][0], transform.m[1][1], transform.m[1][2]);
    Vector3 row2(transform.m[2][0], transform.m[2][1], transform.m[2][2]);
    
    scale.x = row0.length();
    scale.y = row1.length();
    scale.z = row2.length();
    
    // 提取旋转
    Matrix3x3 rotMat;
    for (int i = 0; i < 3; ++i) {
        for (int j = 0; j < 3; ++j) {
            rotMat.m[i][j] = transform.m[i][j];
            if (scale[j] > Math::EPSILON) {
                rotMat.m[i][j] /= scale[j];
            }
        }
    }
    
    rotation = Quaternion(rotMat);
    return true;
}

inline bool MatrixDecomposition::solveLinearSystem(const Matrix4x4& A, const Vector3& b, Vector3& x) {
    Matrix4x4 L, U, P;
    if (!luDecompose(A, L, U, P)) return false;
    
    // 解 Ly = Pb
    float y[4];
    float pb[4] = {b.x, b.y, b.z, 1.0f};
    
    for (int i = 0; i < 4; ++i) {
        y[i] = pb[i];
        for (int j = 0; j < i; ++j) {
            y[i] -= L.m[i][j] * y[j];
        }
    }
    
    // 解 Ux = y
    for (int i = 3; i >= 0; --i) {
        x[i] = y[i];
        for (int j = i + 1; j < 4; ++j) {
            x[i] -= U.m[i][j] * x[j];
        }
        if (std::abs(U.m[i][i]) < Math::EPSILON) return false;
        x[i] /= U.m[i][i];
    }
    
    return true;
}

inline int MatrixDecomposition::computeRank(const Matrix4x4& A, float tolerance) {
    Matrix4x4 U = A;
    int rank = 0;
    
    for (int col = 0; col < 4; ++col) {
        // 找主元
        int pivotRow = col;
        for (int row = col + 1; row < 4; ++row) {
            if (std::abs(U.m[row][col]) > std::abs(U.m[pivotRow][col])) {
                pivotRow = row;
            }
        }
        
        if (std::abs(U.m[pivotRow][col]) < tolerance) continue;
        
        // 交换行
        for (int j = 0; j < 4; ++j) {
            std::swap(U.m[col][j], U.m[pivotRow][j]);
        }
        
        // 消元
        for (int row = col + 1; row < 4; ++row) {
            float factor = U.m[row][col] / U.m[col][col];
            for (int j = col; j < 4; ++j) {
                U.m[row][j] -= factor * U.m[col][j];
            }
        }
        
        rank++;
    }
    
    return rank;
}

inline float MatrixDecomposition::computeConditionNumber(const Matrix4x4& A) {
    // 简化实现：使用Frobenius范数
    float normA = 0.0f;
    float normInv = 0.0f;
    
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            normA += A.m[i][j] * A.m[i][j];
    normA = std::sqrt(normA);
    
    Matrix4x4 A_inv = A.inverse();
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            normInv += A_inv.m[i][j] * A_inv.m[i][j];
    normInv = std::sqrt(normInv);
    
    return normA * normInv;
}

} // namespace AstraAlgebra
