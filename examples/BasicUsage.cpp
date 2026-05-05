// BasicUsage.cpp - 基本用法示例
// 展示核心功能的简单用法

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Core/Transform.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Core/Serialization.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Geometry/Triangle.h>
#include <AstraAlgebra/Geometry/Noise.h>
#include <AstraAlgebra/Geometry/Capsule.h>
#include <AstraAlgebra/Geometry/Cone.h>
#include <AstraAlgebra/Geometry/Cylinder.h>
#include <AstraAlgebra/Geometry/GJK.h>
#include <AstraAlgebra/Geometry/Line.h>
#include <AstraAlgebra/Numerics/NumericalMethods.h>
#include <iostream>
#include <chrono>

using namespace AstraAlgebra;

int main() {
    std::cout << "====================================\n";
    std::cout << "      AstraAlgebra 基本用法示例\n";
    std::cout << "====================================\n\n";
    
    // 1. 向量基本操作
    std::cout << "1. 向量基本操作\n";
    Vector3 v1(1.0f, 2.0f, 3.0f);
    Vector3 v2(4.0f, 5.0f, 6.0f);
    
    std::cout << "v1 = " << v1 << std::endl;
    std::cout << "v2 = " << v2 << std::endl;
    std::cout << "v1 + v2 = " << v1 + v2 << std::endl;
    std::cout << "v1 - v2 = " << v1 - v2 << std::endl;
    std::cout << "v1 * 2.0f = " << v1 * 2.0f << std::endl;
    std::cout << "v1.dot(v2) = " << v1.dot(v2) << std::endl;
    std::cout << "v1.cross(v2) = " << v1.cross(v2) << std::endl;
    std::cout << "v1.length() = " << v1.length() << std::endl;
    std::cout << "v1.normalized() = " << v1.normalized() << std::endl;
    std::cout << std::endl;
    
    // 2. 四元数基本操作
    std::cout << "2. 四元数基本操作\n";
    Quaternion q1 = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    Quaternion q2 = Quaternion::fromEulerAngles(Vector3(30.0f, 45.0f, 60.0f), true);
    
    std::cout << "q1 (90度绕Y轴) = " << q1 << std::endl;
    std::cout << "q2 (Euler角: 30, 45, 60) = " << q2 << std::endl;
    std::cout << "q1 * q2 = " << q1 * q2 << std::endl;
    
    // 使用四元数旋转向量
    Vector3 forward = Vector3::forward;
    Vector3 rotatedForward = q1 * forward;
    std::cout << "forward = " << forward << std::endl;
    std::cout << "q1 * forward = " << rotatedForward << std::endl;
    std::cout << std::endl;
    
    // 3. 矩阵基本操作
    std::cout << "3. 矩阵基本操作\n";
    Matrix4x4 translationMat = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));
    Matrix4x4 rotationMat = Matrix4x4::rotation(Vector3::up, 90.0f, true);
    Matrix4x4 scaleMat = Matrix4x4::scale(Vector3(2.0f, 2.0f, 2.0f));
    
    // 组合变换矩阵
    Matrix4x4 transformMat = translationMat * rotationMat * scaleMat;
    std::cout << "Translation Matrix:\n" << translationMat << std::endl;
    std::cout << "Rotation Matrix (90度绕Y轴):\n" << rotationMat << std::endl;
    std::cout << "Scale Matrix:\n" << scaleMat << std::endl;
    std::cout << "Combined Transform Matrix:\n" << transformMat << std::endl;
    
    // 使用矩阵变换向量
    Vector3 point(1.0f, 0.0f, 0.0f);
    Vector3 transformedPoint = transformMat * point;
    std::cout << "Original Point: " << point << std::endl;
    std::cout << "Transformed Point: " << transformedPoint << std::endl;
    std::cout << std::endl;
    
    // 4. Transform变换系统
    std::cout << "4. Transform变换系统\n";
    Transform transform;
    transform.setPosition(Vector3(1.0f, 0.0f, 0.0f));
    transform.setRotation(q1);
    transform.setScale(Vector3(2.0f, 2.0f, 2.0f));
    
    std::cout << "Transform Position: " << transform.getPosition() << std::endl;
    std::cout << "Transform Rotation: " << transform.getRotation() << std::endl;
    std::cout << "Transform Scale: " << transform.getScale() << std::endl;
    
    // 获取世界变换矩阵
    Matrix4x4 worldMatrix = transform.getWorldMatrix();
    std::cout << "Transform World Matrix:\n" << worldMatrix << std::endl;
    
    // 旋转一个向量
    Vector3 transformedVec = transform.localToWorld(Vector3::forward);
    std::cout << "Local Forward Vector: " << Vector3::forward << std::endl;
    std::cout << "World Forward Vector: " << transformedVec << std::endl;
    std::cout << std::endl;
    
    // 5. 几何工具
    std::cout << "5. 几何工具\n";
    
    // 射线与平面相交
    Ray ray(Vector3(0.0f, 5.0f, 0.0f), Vector3(0.0f, -1.0f, 0.0f));
    Plane plane(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
    
    float t;
    if (plane.intersects(ray, t)) {
        Vector3 intersection = ray.at(t);
        std::cout << "Ray-plane intersection at: " << intersection << std::endl;
    } else {
        std::cout << "Ray does not intersect plane" << std::endl;
    }
    
    // 射线与球体相交
    Sphere sphere(Vector3(0.0f, 0.0f, 0.0f), 5.0f);
    if (sphere.intersects(ray, t)) {
        Vector3 intersection = ray.at(t);
        std::cout << "Ray-sphere intersection at: " << intersection << std::endl;
    } else {
        std::cout << "Ray does not intersect sphere" << std::endl;
    }
    
    // 射线与三角面相交
    Triangle triangle(Vector3(0.0f, 0.0f, -5.0f), Vector3(5.0f, 0.0f, -5.0f), Vector3(0.0f, 5.0f, -5.0f));
    if (triangle.intersects(ray, t)) {
        Vector3 intersection = ray.at(t);
        std::cout << "Ray-triangle intersection at: " << intersection << std::endl;
        
        // 计算三角面法线
        Vector3 normal = triangle.getNormal();
        std::cout << "Triangle normal: " << normal << std::endl;
        
        // 计算三角面面积
        float area = triangle.getArea();
        std::cout << "Triangle area: " << area << std::endl;
    } else {
        std::cout << "Ray does not intersect triangle" << std::endl;
    }
    
    // 点与边界盒
    Bounds bounds(Vector3(-5.0f, -5.0f, -5.0f), Vector3(5.0f, 5.0f, 5.0f));
    Vector3 pointInBounds(1.0f, 1.0f, 1.0f);
    Vector3 pointOutOfBounds(10.0f, 1.0f, 1.0f);
    
    std::cout << "Point " << pointInBounds << " is in bounds: " << (bounds.contains(pointInBounds) ? "true" : "false") << std::endl;
    std::cout << "Point " << pointOutOfBounds << " is in bounds: " << (bounds.contains(pointOutOfBounds) ? "true" : "false") << std::endl;
    
    // 噪声生成示例
    std::cout << "\n6. 噪声生成示例\n";
    Noise noise;
    
    // 2D Perlin噪声
    float noiseValue2D = noise.perlin2D(1.2f, 3.4f);
    std::cout << "2D Perlin noise at (1.2, 3.4): " << noiseValue2D << std::endl;
    
    // 3D Perlin噪声
    float noiseValue3D = noise.perlin3D(1.2f, 3.4f, 5.6f);
    std::cout << "3D Perlin noise at (1.2, 3.4, 5.6): " << noiseValue3D << std::endl;
    
    // 分形噪声（多层叠加）
    float fractalNoise = noise.fractalPerlin2D(1.2f, 3.4f, 4, 0.5f);
    std::cout << "Fractal Perlin noise (4 octaves) at (1.2, 3.4): " << fractalNoise << std::endl;
    
    // 测试不同种子的噪声
    noise.setSeed(12345);
    float noiseValueSeed1 = noise.perlin2D(1.2f, 3.4f);
    noise.setSeed(54321);
    float noiseValueSeed2 = noise.perlin2D(1.2f, 3.4f);
    std::cout << "Noise with seed 12345: " << noiseValueSeed1 << std::endl;
    std::cout << "Noise with seed 54321: " << noiseValueSeed2 << std::endl;
    
    // 7. 线性代数高级功能
    std::cout << "\n7. 线性代数高级功能\n";
    
    // 7.1 矩阵特征值计算
    Matrix3x3 rotationMat3x3 = Matrix3x3::rotationY(45.0f, true);
    std::cout << "Rotation Matrix (45度绕Y轴):\n" << rotationMat3x3 << std::endl;
    
    // 计算特征值和特征向量
    float eigenvalues[3];
    Vector3 eigenvectors[3];
    rotationMat3x3.eigenvectors(eigenvalues, eigenvectors);
    
    std::cout << "Eigenvalues: " << eigenvalues[0] << ", " << eigenvalues[1] << ", " << eigenvalues[2] << std::endl;
    std::cout << "Dominant eigenvector: " << eigenvectors[0] << std::endl;
    
    // 使用幂法计算最大特征值
    Vector3 v;
    float lambda = rotationMat3x3.powerMethod(v);
    std::cout << "Power method result: lambda = " << lambda << ", eigenvector = " << v << std::endl;
    
    // 7.2 AVX/AVX2 性能演示
    std::cout << "\n8. AVX/AVX2 性能演示\n";
    
    // 向量运算性能测试
    const int iterations = 10000000;
    
    // 创建测试向量
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    Vector3 result;
    
    // 计时开始
    auto start = std::chrono::high_resolution_clock::now();
    
    // 执行大量向量加法操作
    for (int i = 0; i < iterations; ++i) {
        result = a + b;
        a = result;
        b = a * 2.0f;
    }
    
    // 计时结束
    auto end = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
    
    std::cout << "执行 " << iterations << " 次向量运算耗时: " << duration << " 毫秒\n";
    std::cout << "结果向量: " << result << std::endl;
    
    // 8. 4D向量示例
    std::cout << "\n9. 4D向量示例\n";
    Vector4 v4_1(1.0f, 2.0f, 3.0f, 4.0f);
    Vector4 v4_2(5.0f, 6.0f, 7.0f, 8.0f);
    
    std::cout << "v4_1 = " << v4_1 << std::endl;
    std::cout << "v4_2 = " << v4_2 << std::endl;
    std::cout << "v4_1 + v4_2 = " << v4_1 + v4_2 << std::endl;
    std::cout << "v4_1.dot(v4_2) = " << v4_1.dot(v4_2) << std::endl;
    
    // 9. 新增几何形状示例
    std::cout << "\n9. 新增几何形状示例\n";
    
    // Capsule
    Capsule capsule(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 2.0f, 0.0f), 1.0f);
    std::cout << "Capsule: 点1(" << capsule.point1 << "), 点2(" << capsule.point2 << "), 半径(" << capsule.radius << ")" << std::endl;
    
    // Cone
    Cone cone(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 3.0f, 0.0f), 1.0f);
    std::cout << "Cone: 顶点(" << cone.apex << "), 底面中心(" << cone.baseCenter << "), 高度(" << cone.getHeight() << "), 底面半径(" << cone.radius << ")" << std::endl;
    
    // Cylinder
    Cylinder cylinder(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 2.0f, 0.0f), 1.0f);
    std::cout << "Cylinder: 底面中心1(" << cylinder.center1 << "), 底面中心2(" << cylinder.center2 << "), 高度(" << cylinder.getHeight() << "), 半径(" << cylinder.radius << ")" << std::endl;
    
    // 10. GJK碰撞检测示例
    std::cout << "\n10. GJK碰撞检测示例\n";
    std::cout << "GJK碰撞检测功能已实现，支持Capsule、Cone、Cylinder等形状" << std::endl;
    
    // 11. 数值方法示例
    std::cout << "\n11. 数值方法示例\n";
    std::cout << "插值功能已实现，支持Linear、CubicSpline、CatmullRom、Bezier、Hermite等插值方法" << std::endl;
    std::cout << "数值积分功能已实现，支持Euler、Verlet、RungeKutta4、Midpoint等方法" << std::endl;
    
    // 12. 增强的数学工具函数示例
    std::cout << "\n12. 增强的数学工具函数示例\n";
    
    // 双曲函数
    float sinhVal = Math::sinh(1.0f);
    float coshVal = Math::cosh(1.0f);
    std::cout << "sinh(1.0f) = " << sinhVal << std::endl;
    std::cout << "cosh(1.0f) = " << coshVal << std::endl;
    
    // 高斯分布随机数
    float gaussianVal = Math::randomGaussian(0.0f, 1.0f);
    std::cout << "Gaussian random number: " << gaussianVal << std::endl;
    
    // 颜色转换（RGB到HSV）
    float rgbR = 0.5f, rgbG = 0.2f, rgbB = 0.8f;
    float h, s, hsvV;
    Math::rgbToHsv(rgbR, rgbG, rgbB, h, s, hsvV);
    std::cout << "RGB(" << rgbR << ", " << rgbG << ", " << rgbB << ") -> HSV(" << h << ", " << s << ", " << hsvV << ")" << std::endl;
    
    // HSV到RGB
    float rgbR2, rgbG2, rgbB2;
    Math::hsvToRgb(h, s, hsvV, rgbR2, rgbG2, rgbB2);
    std::cout << "HSV(" << h << ", " << s << ", " << hsvV << ") -> RGB(" << rgbR2 << ", " << rgbG2 << ", " << rgbB2 << ")" << std::endl;
    
    // 13. 序列化支持示例
    std::cout << "\n13. 序列化支持示例\n";
    
    // 序列化Vector3为字符串格式
    Vector3 vec(1.0f, 2.0f, 3.0f);
    std::string serialized = Serialization::serialize(vec, Serialization::Format::String);
    std::cout << "Vector3序列化结果: " << serialized << std::endl;
    
    // 反序列化
    Vector3 deserialized = Serialization::deserialize<Vector3>(serialized, Serialization::Format::String);
    std::cout << "Vector3反序列化结果: " << deserialized << std::endl;
    
    // 序列化Matrix4x4为二进制格式
    Matrix4x4 mat = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));
    std::string serializedBinary = Serialization::serialize(mat, Serialization::Format::Binary);
    std::cout << "Matrix4x4二进制序列化结果长度: " << serializedBinary.size() << " 字节" << std::endl;
    
    // 反序列化二进制数据
    Matrix4x4 deserializedBinary = Serialization::deserialize<Matrix4x4>(serializedBinary, Serialization::Format::Binary);
    std::cout << "Matrix4x4二进制反序列化结果: " << std::endl << deserializedBinary << std::endl;
    
    // 序列化四元数
    Quaternion quat = Quaternion::fromAxisAngle(Vector3::up, 45.0f, true);
    std::string serializedQuat = Serialization::serialize(quat);
    std::cout << "Quaternion序列化结果: " << serializedQuat << std::endl;
    
    Quaternion deserializedQuat = Serialization::deserialize<Quaternion>(serializedQuat);
    std::cout << "Quaternion反序列化结果: " << deserializedQuat << std::endl;
    
    // 14. 线段类示例
    std::cout << "\n14. 线段类示例\n";
    
    // 创建线段
    Line line(Vector3(0.0f, 0.0f, 0.0f), Vector3(5.0f, 5.0f, 5.0f));
    std::cout << "线段起点: " << line.start << std::endl;
    std::cout << "线段终点: " << line.end << std::endl;
    std::cout << "线段长度: " << line.length() << std::endl;
    std::cout << "线段中点: " << line.midpoint() << std::endl;
    std::cout << "线段方向: " << line.direction() << std::endl;
    std::cout << "线段上t=0.5的点: " << line.at(0.5f) << std::endl;
    
    // 测试点到线段的距离
    Vector3 testPoint(1.0f, 2.0f, 3.0f);
    float distance = line.distanceTo(testPoint);
    std::cout << "点 " << testPoint << " 到线段的距离: " << distance << std::endl;
    
    // 测试线段与平面的相交
    Plane testPlane(Vector3(1.0f, 1.0f, 1.0f).normalized(), 2.0f);
    float tLine;
    if (line.intersects(testPlane, tLine)) {
        Vector3 intersection = line.at(tLine);
        std::cout << "线段与平面相交于点: " << intersection << std::endl;
    } else {
        std::cout << "线段与平面不相交" << std::endl;
    }
    
    // 测试线段与线段的相交
    Line line2(Vector3(0.0f, 5.0f, 0.0f), Vector3(5.0f, 0.0f, 5.0f));
    Vector3 lineIntersection;
    if (line.intersects(line2, lineIntersection)) {
        std::cout << "两条线段相交于点: " << lineIntersection << std::endl;
    } else {
        std::cout << "两条线段不相交" << std::endl;
    }
    
    std::cout << std::endl;
    std::cout << "====================================\n";
    std::cout << "        示例程序结束\n";
    std::cout << "====================================\n";
    
    return 0;
}