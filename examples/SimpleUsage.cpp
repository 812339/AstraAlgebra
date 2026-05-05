// AstraAlgebra 数学库简单使用示例
// 展示如何在实际项目中使用向量、矩阵、四元数等功能

#include <iostream>
#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Core/MathUtils.h>

using namespace AstraAlgebra;

int main() {
    std::cout << "=== AstraAlgebra 数学库使用示例 ===" << std::endl;
    
    // 1. 向量基本操作
    std::cout << "\n1. 向量基本操作:" << std::endl;
    
    Vector2 v2(1.0f, 2.0f);
    Vector3 v3(1.0f, 2.0f, 3.0f);
    Vector4 v4(1.0f, 2.0f, 3.0f, 4.0f);
    
    std::cout << "Vector2: (" << v2.x << ", " << v2.y << ")" << std::endl;
    std::cout << "Vector3: (" << v3.x << ", " << v3.y << ", " << v3.z << ")" << std::endl;
    std::cout << "Vector4: (" << v4.x << ", " << v4.y << ", " << v4.z << ", " << v4.w << ")" << std::endl;
    
    // 向量运算
    Vector3 a(1.0f, 0.0f, 0.0f);
    Vector3 b(0.0f, 1.0f, 0.0f);
    Vector3 c = a + b;
    Vector3 d = Vector3::cross(a, b);
    
    std::cout << "a + b = (" << c.x << ", " << c.y << ", " << c.z << ")" << std::endl;
    std::cout << "a × b = (" << d.x << ", " << d.y << ", " << d.z << ")" << std::endl;
    std::cout << "a · b = " << a.dot(b) << std::endl;
    
    // 2. 矩阵变换
    std::cout << "\n2. 矩阵变换:" << std::endl;
    
    // 创建平移矩阵
    Matrix4x4 translation = Matrix4x4::translation(Vector3(1.0f, 0.0f, 0.0f));
    
    // 创建旋转矩阵
    Matrix4x4 rotation = Matrix4x4::rotationY(Math::degToRad(45.0f));
    
    // 组合变换
    Matrix4x4 transform = translation * rotation;
    
    // 变换点
    Vector4 point(0.0f, 0.0f, 0.0f, 1.0f);
    Vector4 transformed = transform * point;
    
    std::cout << "原始点: (" << point.x << ", " << point.y << ", " << point.z << ")" << std::endl;
    std::cout << "变换后: (" << transformed.x << ", " << transformed.y << ", " << transformed.z << ")" << std::endl;
    
    // 3. 四元数旋转
    std::cout << "\n3. 四元数旋转:" << std::endl;
    
    // 创建绕Y轴旋转90度的四元数
    Quaternion rotationQuat = Quaternion::fromAxisAngle(Vector3(0.0f, 1.0f, 0.0f), 90.0f);
    
    // 旋转向量
    Vector3 original(1.0f, 0.0f, 0.0f);
    Vector3 rotated = rotationQuat * original;
    
    std::cout << "原始向量: (" << original.x << ", " << original.y << ", " << original.z << ")" << std::endl;
    std::cout << "旋转后: (" << rotated.x << ", " << rotated.y << ", " << rotated.z << ")" << std::endl;
    
    // 4. 几何相交检测
    std::cout << "\n4. 几何相交检测:" << std::endl;
    
    // 创建射线
    Ray ray(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f));
    
    // 创建球体
    Sphere sphere(Vector3(5.0f, 0.0f, 0.0f), 1.0f);
    
    // 检测相交
    float distance = ray.intersect(sphere);
    
    if (distance >= 0.0f) {
        std::cout << "射线与球体相交，距离: " << distance << std::endl;
        Vector3 intersection = ray.getPoint(distance);
        std::cout << "交点: (" << intersection.x << ", " << intersection.y << ", " << intersection.z << ")" << std::endl;
    } else {
        std::cout << "射线与球体不相交" << std::endl;
    }
    
    // 5. 数学工具函数
    std::cout << "\n5. 数学工具函数:" << std::endl;
    
    // 角度转换
    float radians = Math::degToRad(180.0f);
    float degrees = Math::radToDeg(Math::PI);
    
    std::cout << "180度 = " << radians << " 弧度" << std::endl;
    std::cout << "π弧度 = " << degrees << " 度" << std::endl;
    
    // 插值
    float lerpResult = Math::lerp(0.0f, 10.0f, 0.5f);
    std::cout << "线性插值(0, 10, 0.5) = " << lerpResult << std::endl;
    
    // 6. 向量插值和归一化
    std::cout << "\n6. 向量插值和归一化:" << std::endl;
    
    Vector3 start(0.0f, 0.0f, 0.0f);
    Vector3 end(10.0f, 0.0f, 0.0f);
    
    Vector3 lerped = Vector3::lerp(start, end, 0.3f);
    Vector3 normalized = end.normalized();
    
    std::cout << "线性插值: (" << lerped.x << ", " << lerped.y << ", " << lerped.z << ")" << std::endl;
    std::cout << "归一化向量: (" << normalized.x << ", " << normalized.y << ", " << normalized.z << ")" << std::endl;
    std::cout << "归一化向量长度: " << normalized.length() << std::endl;
    
    // 7. 矩阵分解
    std::cout << "\n7. 矩阵分解:" << std::endl;
    
    Vector3 translationVec, scaleVec;
    Quaternion rotationQuat2;
    transform.decompose(translationVec, rotationQuat2, scaleVec);
    
    std::cout << "平移: (" << translationVec.x << ", " << translationVec.y << ", " << translationVec.z << ")" << std::endl;
    std::cout << "缩放: (" << scaleVec.x << ", " << scaleVec.y << ", " << scaleVec.z << ")" << std::endl;
    
    std::cout << "\n=== 示例结束 ===" << std::endl;
    
    return 0;
}