// Plane - 平面，ax + by + cz + d = 0 那种

#pragma once

#include <AstraAlgebra/Core/MathConstants.h>
#include <AstraAlgebra/Vector/Vector3.h>

namespace AstraAlgebra {

// 前向声明
class Ray;
class Sphere;
class Bounds;

// 平面类，用一般式表示的，法向量加距离
class Plane {
public:
    float a, b, c, d;  // ax + by + cz + d = 0 的系数

    // 构造函数
    Plane();  // 默认，xy平面（z=0）
    Plane(float a, float b, float c, float d);  // 直接给系数
    Plane(const Vector3& normal, float distance);  // 法向量加距离
    Plane(const Vector3& normal, const Vector3& point);  // 法向量加平面上一点
    Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2);  // 三个点定一个平面
    Plane(const Plane& other);  // 拷贝构造
    Plane(Plane&& other) noexcept;  // 移动构造

    ~Plane() = default;

    // 赋值
    Plane& operator=(const Plane& other);
    Plane& operator=(Plane&& other) noexcept;

    // 平面方法
    Vector3 getNormal() const;  // 拿法向量
    float getDistance() const;  // 平面到原点的距离
    Plane normalized() const;  // 返回归一化后的平面
    Plane& normalize();  // 归一化自己
    float distanceToPoint(const Vector3& point) const;  // 点到平面的距离
    Vector3 projectPoint(const Vector3& point) const;  // 把点投影到平面上
    bool isFrontFacing(const Vector3& direction) const;  // 判断方向是不是朝正面

    // 碰撞检测
    bool intersects(const Ray& ray, float& t) const;  // 射线相交
    bool intersects(const Vector3& point) const;  // 点在不在平面上
    bool intersects(const Plane& other, Ray& intersection) const;  // 两个平面相交，交线是射线
    bool intersects(const Sphere& sphere) const;  // 跟球碰不碰
    bool intersects(const Bounds& bounds) const;  // 跟AABB碰不碰

    // 静态方法
    static Plane fromPoints(const Vector3& p0, const Vector3& p1, const Vector3& p2);  // 三点创建平面
    static Plane xyPlane();  // xy平面
    static Plane xzPlane();  // xz平面
    static Plane yzPlane();  // yz平面
};

} // namespace AstraAlgebra
