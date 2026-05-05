// Line - 线段，两点之间的线段，碰撞检测常用

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Triangle.h>
#include <AstraAlgebra/Geometry/Bounds.h>

namespace AstraAlgebra {

// 线段类，两点之间连一条线
class Line {
public:
    Vector3 start;
    Vector3 end;
    
    // 构造函数
    Line() = default;
    Line(const Vector3& start, const Vector3& end);
    
    // 基本属性
    float length() const;           // 长度
    float lengthSquared() const;    // 长度平方
    Vector3 direction() const;      // 方向（归一化）
    Vector3 midpoint() const;       // 中点
    Vector3 at(float t) const;      // t在[0,1]之间的点
    
    // 距离计算
    float distanceTo(const Vector3& point) const;           // 点到线段距离
    float distanceSquaredTo(const Vector3& point) const;    // 点到线段距离平方
    Vector3 closestPointTo(const Vector3& point) const;     // 线段上离点最近的点
    
    // 相交检测
    bool intersects(const Ray& ray, float& tLine, float& tRay) const;           // 跟射线
    bool intersects(const Plane& plane, float& t) const;                        // 跟平面
    bool intersects(const Sphere& sphere) const;                                // 跟球
    bool intersects(const Triangle& triangle, Vector3& intersectionPoint) const; // 跟三角形
    bool intersects(const Bounds& bounds) const;                                // 跟AABB
    bool intersects(const Line& other, Vector3& intersectionPoint) const;       // 跟另一条线段
    
    // 其他操作
    Ray toRay() const;                                                          // 转射线
    bool contains(const Vector3& point) const;                                  // 包不包含点
    Line reversed() const;                                                      // 反转方向
    Line extended(float startExtension, float endExtension) const;              // 延长
    Bounds bounds() const;                                                      // 包围盒
    
    // 静态方法
    static Line fromPointDirection(const Vector3& point, const Vector3& direction, float length); // 从点+方向创建
    static Line fromPoints(const Vector3* points, size_t count);                // 从点集创建
};

} // namespace AstraAlgebra