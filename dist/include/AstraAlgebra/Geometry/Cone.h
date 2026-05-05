// Cone - 圆锥，顶点加底面，碰撞检测用

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Bounds.h>

namespace AstraAlgebra {

// 圆锥类，顶点、底面中心加半径
class Cone {
public:
    Vector3 apex;     // 顶点
    Vector3 baseCenter; // 底面中心
    float radius;     // 底面半径
    
    // 构造函数
    Cone();  // 默认
    Cone(const Vector3& apex, const Vector3& baseCenter, float radius); // 顶点、底面中心加半径
    Cone(const Cone& other);  // 拷贝构造
    Cone(Cone&& other) noexcept;  // 移动构造
    
    // 赋值
    Cone& operator=(const Cone& other);
    Cone& operator=(Cone&& other) noexcept;
    
    // 圆锥方法
    float getHeight() const;  // 高度
    Vector3 getAxis() const;  // 轴线向量
    float getSlantHeight() const;  // 斜高
    float getAngle() const;  // 半顶角（弧度）
    Sphere getSphere() const;  // 包围球
    Bounds getBounds() const;  // 包围盒
    Vector3 closestPoint(const Vector3& point) const;  // 圆锥上离给点最近的点
    float distanceToPoint(const Vector3& point) const;  // 点到圆锥的距离
    
    // 碰撞检测
    bool intersects(const Ray& ray, float& t) const;  // 射线相交
    bool intersects(const Vector3& point) const;  // 点在不在圆锥内
    bool intersects(const Sphere& sphere) const;  // 跟球碰不碰
    bool intersects(const Plane& plane) const;  // 跟平面碰不碰
    bool intersects(const Bounds& bounds) const;  // 跟AABB碰不碰
    
    // 静态方法
    static Cone fromPoints(const Vector3& apex, const Vector3& baseCenter, float radius);  // 从点创建
    static Cone vertical(float height, float radius);  // 垂直的（沿Y轴，顶点向上）
    static Cone verticalDown(float height, float radius);  // 垂直的（沿Y轴，顶点向下）
    static Cone horizontalX(float length, float radius);  // 水平的（沿X轴）
    static Cone horizontalZ(float length, float radius);  // 水平的（沿Z轴）
};

} // namespace AstraAlgebra
