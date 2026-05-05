// Sphere - 球体，游戏里做碰撞检测经常用

#pragma once

#include <AstraAlgebra/Core/MathConstants.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>

namespace AstraAlgebra {

// 前向声明
class Ray;
class Plane;
class Bounds;

// 球体类，中心加半径，简单好用
class Sphere {
public:
    Vector3 center;  // 球心
    float radius;    // 半径

    // 构造函数
    Sphere();  // 默认，中心在原点，半径1
    Sphere(const Vector3& center, float radius);  // 给中心和半径就行
    Sphere(const Sphere& other);  // 拷贝构造
    Sphere(Sphere&& other) noexcept;  // 移动构造
    Sphere(const Vector3& point);  // 从一个点搞出来（半径0）
    Sphere(const Vector3& p0, const Vector3& p1);  // 两点当直径
    Sphere(const Vector3* points, size_t count);  // 一堆点，算包围球
    Sphere(const Bounds& bounds);  // 从AABB搞个球

    ~Sphere() = default;

    // 赋值
    Sphere& operator=(const Sphere& other);
    Sphere& operator=(Sphere&& other) noexcept;

    // 球体方法
    float volume() const;  // 体积
    float surfaceArea() const;  // 表面积
    bool contains(const Vector3& point) const;  // 点在不在球里面
    bool contains(const Sphere& other) const;  // 包不包含另一个球
    bool contains(const Bounds& bounds) const;  // 包不包含AABB
    Sphere merged(const Sphere& other) const;  // 合并两个球，返回新的
    Sphere& merge(const Sphere& other);  // 合并到当前球
    Sphere transformed(const Matrix4x4& transform) const;  // 变换一下

    // 碰撞检测
    bool intersects(const Ray& ray, float& t) const;  // 射线相交
    bool intersects(const Vector3& point) const;  // 点在不在球面上
    bool intersects(const Sphere& other) const;  // 跟另一个球碰不碰
    bool intersects(const Plane& plane) const;  // 跟平面碰不碰
    bool intersects(const Bounds& bounds) const;  // 跟AABB碰不碰

    // 静态方法
    static Sphere fromPoints(const Vector3* points, size_t count);  // 从一堆点搞个包围球
    static Sphere boundingSphere(const Bounds& bounds);  // 从AABB搞个包围球
};

} // namespace AstraAlgebra
