// Cylinder - 圆柱，两个底面中心加半径，碰撞检测用

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Bounds.h>

namespace AstraAlgebra {

// 圆柱类，两个底面中心加半径
class Cylinder {
public:
    Vector3 center1; // 底面中心1
    Vector3 center2; // 底面中心2
    float radius;    // 半径
    
    // 构造函数
    Cylinder();  // 默认
    Cylinder(const Vector3& center1, const Vector3& center2, float radius); // 两个中心加半径
    Cylinder(const Cylinder& other);  // 拷贝构造
    Cylinder(Cylinder&& other) noexcept;  // 移动构造
    
    // 赋值
    Cylinder& operator=(const Cylinder& other);
    Cylinder& operator=(Cylinder&& other) noexcept;
    
    // 圆柱方法
    float getHeight() const;  // 高度
    Vector3 getAxis() const;  // 轴线向量
    Sphere getSphere() const;  // 包围球
    Bounds getBounds() const;  // 包围盒
    Vector3 closestPoint(const Vector3& point) const;  // 圆柱上离给点最近的点
    float distanceToPoint(const Vector3& point) const;  // 点到圆柱的距离
    
    // 碰撞检测
    bool intersects(const Ray& ray, float& t) const;  // 射线相交
    bool intersects(const Vector3& point) const;  // 点在不在圆柱内
    bool intersects(const Sphere& sphere) const;  // 跟球碰不碰
    bool intersects(const Plane& plane) const;  // 跟平面碰不碰
    bool intersects(const Bounds& bounds) const;  // 跟AABB碰不碰
    
    // 静态方法
    static Cylinder fromPoints(const Vector3& center1, const Vector3& center2, float radius);  // 从点创建
    static Cylinder vertical(float height, float radius);  // 垂直的（沿Y轴）
    static Cylinder horizontalX(float length, float radius);  // 水平的（沿X轴）
    static Cylinder horizontalZ(float length, float radius);  // 水平的（沿Z轴）
};

} // namespace AstraAlgebra
