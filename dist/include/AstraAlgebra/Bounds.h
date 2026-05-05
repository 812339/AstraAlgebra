// Bounds - AABB，轴对齐包围盒，碰撞检测用得很多

#pragma once

#include <AstraAlgebra/Core/MathConstants.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>

namespace AstraAlgebra {

// 前向声明
class Ray;
class Sphere;
class Plane;

// 边界框类，就是min和max两个点，游戏里天天用
class Bounds {
public:
    Vector3 min;  // 最小点
    Vector3 max;  // 最大点

    // 构造函数
    Bounds();  // 默认，无效的框
    Bounds(const Vector3& min, const Vector3& max);  // 给min和max就行
    Bounds(const Vector3& point);  // 一个点搞出来
    Bounds(const Bounds& other);  // 拷贝构造
    Bounds(Bounds&& other) noexcept;  // 移动构造
    Bounds(const Vector3* points, size_t count);  // 一堆点，算包围盒
    Bounds(const Sphere& sphere);  // 从球搞个盒

    ~Bounds() = default;

    // 赋值
    Bounds& operator=(const Bounds& other);
    Bounds& operator=(Bounds&& other) noexcept;

    // 边界框方法
    Vector3 getCenter() const;  // 中心点
    Vector3 getExtents() const;  // 半长
    Vector3 getSize() const;  // 尺寸
    float volume() const;  // 体积
    float surfaceArea() const;  // 表面积
    bool isValid() const;  // 有效不
    void invalidate();  // 搞成无效
    bool contains(const Vector3& point) const;  // 点在不在里面
    bool contains(const Bounds& other) const;  // 包不包含另一个框
    bool contains(const Sphere& sphere) const;  // 包不包含球
    Bounds merged(const Bounds& other) const;  // 合并两个框，返回新的
    Bounds& merge(const Bounds& other);  // 合并到当前框
    Bounds merged(const Vector3& point) const;  // 框跟点合并
    Bounds& merge(const Vector3& point);  // 框跟点合并到自己
    Vector3 closestPoint(const Vector3& point) const;  // 框上离给点最近的点
    Bounds transformed(const Matrix4x4& transform) const;  // 变换一下

    // 碰撞检测
    bool intersects(const Ray& ray, float& t) const;  // 射线相交
    bool intersects(const Bounds& other) const;  // 跟另一个框碰不碰
    bool intersects(const Sphere& sphere) const;  // 跟球碰不碰
    bool intersects(const Plane& plane) const;  // 跟平面碰不碰

    // 静态方法
    static Bounds fromPoints(const Vector3* points, size_t count);  // 从一堆点搞包围盒
    static Bounds fromSphere(const Sphere& sphere);  // 从球搞包围盒
    static Bounds infinity();  // 无限大的框
    static Bounds zero();  // 零大小的框
};

} // namespace AstraAlgebra
