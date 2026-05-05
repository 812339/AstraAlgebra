// Ray - 射线，起点加方向，碰撞检测、光线追踪都离不开它

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>

namespace AstraAlgebra {

// 射线类，起点加方向，碰撞检测、光线追踪都用它
class Ray {
public:
    Vector3 origin;    // 起点
    Vector3 direction; // 方向（记得归一化）
    
    // 构造函数
    Ray();                          // 默认构造
    Ray(const Vector3& origin, const Vector3& direction); // 起点加方向
    Ray(const Ray& other);          // 拷贝构造
    Ray(Ray&& other) noexcept;      // 移动构造
    
    // 赋值
    Ray& operator=(const Ray& other);      // 拷贝赋值
    Ray& operator=(Ray&& other) noexcept;  // 移动赋值
    
    // 射线操作
    Vector3 pointAt(float t) const;  // 算射线上t位置的点
    Vector3 at(float t) const;        // 同上，别名而已
    
    // 相交检测
    bool intersects(const class Sphere& sphere, float& t) const;        // 跟球
    bool intersects(const class Plane& plane, float& t) const;          // 跟平面
    bool intersects(const class Triangle& triangle, float& t) const;    // 跟三角形
    bool intersects(const class Bounds& bounds, float& t) const;        // 跟AABB
    
    // 便利方法：直接返回距离，不相交返回-1
    float intersect(const class Sphere& sphere) const;               // 跟球
    float intersect(const class Plane& plane) const;                  // 跟平面
    float intersect(const class Triangle& triangle) const;            // 跟三角形
    float intersect(const class Bounds& bounds) const;                // 跟AABB
};

} // namespace AstraAlgebra