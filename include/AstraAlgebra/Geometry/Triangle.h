// Triangle - 三角形，3D建模和碰撞检测都离不开它

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Bounds.h>

namespace AstraAlgebra {

// 三角形类，三个顶点搞定
class Triangle {
public:
    Vector3 a, b, c; // 三个顶点
    
    // 构造函数
    Triangle();  // 默认
    Triangle(const Vector3& a, const Vector3& b, const Vector3& c); // 给三个点就行
    Triangle(const Triangle& other);  // 拷贝构造
    Triangle(Triangle&& other) noexcept;  // 移动构造
    
    // 赋值
    Triangle& operator=(const Triangle& other);
    Triangle& operator=(Triangle&& other) noexcept;
    
    // 三角形方法
    Vector3 getNormal() const;  // 法线
    float getArea() const;  // 面积
    Bounds getBounds() const;  // 包围盒
    Sphere getSphere() const;  // 包围球
    Plane getPlane() const;  // 所在的平面
    
    // 点跟三角形的关系
    bool contains(const Vector3& point) const;  // 点在不在三角形里
    Vector3 closestPoint(const Vector3& point) const;  // 三角形上离给点最近的点
    float distanceToPoint(const Vector3& point) const;  // 点到三角形的距离
    
    // 碰撞检测
    bool intersects(const Ray& ray, float& t, Vector3& barycentric) const;  // 射线相交，返回重心坐标
    bool intersects(const Ray& ray, float& t) const;  // 射线相交
    bool intersects(const Sphere& sphere) const;  // 跟球碰不碰
    bool intersects(const Plane& plane) const;  // 跟平面碰不碰
    bool intersects(const Bounds& bounds) const;  // 跟AABB碰不碰
    bool intersects(const Triangle& other) const;  // 跟另一个三角形碰不碰
    
    // 静态方法
    static Triangle fromPoints(const Vector3& a, const Vector3& b, const Vector3& c);  // 三点创建三角形
    static float barycentricToArea(const Vector3& barycentric, float triangleArea);  // 重心坐标转面积
};

} // namespace AstraAlgebra
