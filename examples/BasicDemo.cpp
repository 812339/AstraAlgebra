// AstraAlgebra 数学库基本使用演示
// 展示核心功能的简单用法

#include <iostream>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>

using namespace AstraAlgebra;

int main() {
    std::cout << "=== AstraAlgebra Math Library Demo ===" << std::endl;
    
    // 1. 向量基本操作
    std::cout << "\n1. Vector Operations:" << std::endl;
    
    Vector3 v1(1.0f, 0.0f, 0.0f);
    Vector3 v2(0.0f, 1.0f, 0.0f);
    
    std::cout << "v1 = (" << v1.x << ", " << v1.y << ", " << v1.z << ")" << std::endl;
    std::cout << "v2 = (" << v2.x << ", " << v2.y << ", " << v2.z << ")" << std::endl;
    
    // 向量运算
    Vector3 v3 = v1 + v2;
    Vector3 v4 = v1 - v2;
    Vector3 v5 = v1 * 2.0f;
    float dot = v1.dot(v2);
    Vector3 cross = Vector3::cross(v1, v2);
    
    std::cout << "v1 + v2 = (" << v3.x << ", " << v3.y << ", " << v3.z << ")" << std::endl;
    std::cout << "v1 - v2 = (" << v4.x << ", " << v4.y << ", " << v4.z << ")" << std::endl;
    std::cout << "v1 * 2.0 = (" << v5.x << ", " << v5.y << ", " << v5.z << ")" << std::endl;
    std::cout << "v1 . v2 = " << dot << std::endl;
    std::cout << "v1 x v2 = (" << cross.x << ", " << cross.y << ", " << cross.z << ")" << std::endl;
    std::cout << "Length of v1 = " << v1.length() << std::endl;
    
    // 2. 矩阵变换
    std::cout << "\n2. Matrix Transformations:" << std::endl;
    
    // 平移矩阵
    Matrix4x4 translation = Matrix4x4::translation(Vector3(1.0f, 0.0f, 0.0f));
    
    // 旋转矩阵 (绕Y轴旋转90度)
    Matrix4x4 rotation = Matrix4x4::rotationY(Math::degToRad(90.0f));
    
    // 组合变换
    Matrix4x4 transform = translation * rotation;
    
    // 变换点
    Vector4 point(0.0f, 0.0f, 0.0f, 1.0f);
    Vector4 transformed = transform * point;
    
    std::cout << "Original point: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    std::cout << "Transformed point: (" << transformed.x << ", " << transformed.y << ", " << transformed.z << ")" << std::endl;
    
    // 3. 四元数旋转
    std::cout << "\n3. Quaternion Rotation:" << std::endl;
    
    // 创建绕Y轴旋转90度的四元数
    Quaternion q = Quaternion::fromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
    
    // 旋转向量
    Vector3 original(1.0f, 0.0f, 0.0f);
    Vector3 rotated = q * original;
    
    std::cout << "Original vector: (" << original.x << ", " << original.y << ", " << original.z << ")" << std::endl;
    std::cout << "Rotated vector: (" << rotated.x << ", " << rotated.y << ", " << rotated.z << ")" << std::endl;
    
    // 4. 向量插值
    std::cout << "\n4. Vector Interpolation:" << std::endl;
    
    Vector3 start(0.0f, 0.0f, 0.0f);
    Vector3 end(10.0f, 0.0f, 0.0f);
    
    Vector3 lerped = Vector3::lerp(start, end, 0.3f);
    Vector3 normalized = end.normalized();
    
    std::cout << "Start: (" << start.x << ", " << start.y << ", " << start.z << ")" << std::endl;
    std::cout << "End: (" << end.x << ", " << end.y << ", " << end.z << ")" << std::endl;
    std::cout << "Lerp at 0.3: (" << lerped.x << ", " << lerped.y << ", " << lerped.z << ")" << std::endl;
    std::cout << "Normalized end: (" << normalized.x << ", " << normalized.y << ", " << normalized.z << ")" << std::endl;
    std::cout << "Length of normalized vector: " << normalized.length() << std::endl;
    
    std::cout << "\n=== Demo Completed ===" << std::endl;
    
    return 0;
}