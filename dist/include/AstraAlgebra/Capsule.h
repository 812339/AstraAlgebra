// Capsule - 胶囊体，两个半球加中间圆柱，角色碰撞经常用

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Bounds.h>

namespace AstraAlgebra {

// 胶囊体类，两个端点加半径，角色碰撞检测最爱
class Capsule {
public:
    Vector3 point1;  // 端点1
    Vector3 point2;  // 端点2
    float radius;    // 半径
    
    // 构造函数
    Capsule();  // 默认
    Capsule(const Vector3& point1, const Vector3& point2, float radius); // 两个端点加半径
    Capsule(const Capsule& other);  // 拷贝构造
    Capsule(Capsule&& other) noexcept;  // 移动构造
    
    // 赋值
    Capsule& operator=(const Capsule& other);
    Capsule& operator=(Capsule&& other) noexcept;
    
    // 胶囊体方法
    float getLength() const;  // 中间线段长度
    Vector3 getAxis() const;  // 轴线向量
    Sphere getSphere() const;  // 包围球
    Bounds getBounds() const;  // 包围盒
    Vector3 closestPoint(const Vector3& point) const;  // 胶囊体上离给点最近的点
    float distanceToPoint(const Vector3& point) const;  // 点到胶囊体的距离
    
    // 碰撞检测
    bool intersects(const Ray& ray, float& t) const;  // 射线相交
    bool intersects(const Vector3& point) const;  // 点在不在胶囊体内
    bool intersects(const Sphere& sphere) const;  // 跟球碰不碰
    bool intersects(const Plane& plane) const;  // 跟平面碰不碰
    bool intersects(const Bounds& bounds) const;  // 跟AABB碰不碰
    
    // 静态方法
    static Capsule fromPoints(const Vector3& point1, const Vector3& point2, float radius);  // 从端点创建
    static Capsule vertical(float height, float radius);  // 垂直的（沿Y轴）
    static Capsule horizontalX(float length, float radius);  // 水平的（沿X轴）
    static Capsule horizontalZ(float length, float radius);  // 水平的（沿Z轴）
};

} // namespace AstraAlgebra
