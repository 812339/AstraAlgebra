// Sphere.cpp - 球体类实现，碰撞检测经常用

#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>


namespace AstraAlgebra {

// 构造函数
Sphere::Sphere() : center(Vector3::zero), radius(1.0f) {
}

Sphere::Sphere(const Vector3& center, float radius) : center(center), radius(radius) {
}

Sphere::Sphere(const Sphere& other) : center(other.center), radius(other.radius) {
}

Sphere::Sphere(Sphere&& other) noexcept : center(std::move(other.center)), radius(other.radius) {
    other.radius = 0.0f;
}

Sphere::Sphere(const Vector3& point) : center(point), radius(0.0f) {
}

Sphere::Sphere(const Vector3& p0, const Vector3& p1) {
    center = (p0 + p1) * 0.5f;
    radius = (p1 - p0).length() * 0.5f;
}

Sphere::Sphere(const Vector3* points, size_t count) {
    // 使用Ritter算法计算包围球
    if (count == 0) {
        *this = Sphere();
        return;
    }
    
    // 找到初始包围球
    Vector3 min = points[0];
    Vector3 max = points[0];
    
    for (size_t i = 1; i < count; ++i) {
        if (points[i].x < min.x) min.x = points[i].x;
        if (points[i].y < min.y) min.y = points[i].y;
        if (points[i].z < min.z) min.z = points[i].z;
        if (points[i].x > max.x) max.x = points[i].x;
        if (points[i].y > max.y) max.y = points[i].y;
        if (points[i].z > max.z) max.z = points[i].z;
    }
    
    center = (min + max) * 0.5f;
    radius = (max - center).length();
    
    // 扩展包围球以包含所有点
    for (size_t i = 0; i < count; ++i) {
        Vector3 toPoint = points[i] - center;
        float distSq = toPoint.lengthSquared();
        
        if (distSq > radius * radius) {
            float dist = std::sqrt(distSq);
            float newRadius = (radius + dist) * 0.5f;
            float scale = (newRadius - radius) / dist;
            radius = newRadius;
            center += toPoint * scale;
        }
    }
}

Sphere::Sphere(const Bounds& bounds) {
    center = bounds.getCenter();
    radius = (bounds.max - center).length();
}

// 赋值操作符
Sphere& Sphere::operator=(const Sphere& other) {
    if (this != &other) {
        center = other.center;
        radius = other.radius;
    }
    return *this;
}

Sphere& Sphere::operator=(Sphere&& other) noexcept {
    if (this != &other) {
        center = std::move(other.center);
        radius = other.radius;
        other.radius = 0.0f;
    }
    return *this;
}

// 球体方法
float Sphere::volume() const {
    return (4.0f / 3.0f) * Math::PI * radius * radius * radius;
}

float Sphere::surfaceArea() const {
    return 4.0f * Math::PI * radius * radius;
}

bool Sphere::contains(const Vector3& point) const {
    return (point - center).lengthSquared() <= radius * radius;
}

bool Sphere::contains(const Sphere& other) const {
    float dist = (center - other.center).length();
    return dist + other.radius <= radius;
}

bool Sphere::contains(const Bounds& bounds) const {
    // 检查AABB的所有八个顶点是否都在球体内
    Vector3 min = bounds.min;
    Vector3 max = bounds.max;
    
    return contains(Vector3(min.x, min.y, min.z)) &&
           contains(Vector3(min.x, min.y, max.z)) &&
           contains(Vector3(min.x, max.y, min.z)) &&
           contains(Vector3(min.x, max.y, max.z)) &&
           contains(Vector3(max.x, min.y, min.z)) &&
           contains(Vector3(max.x, min.y, max.z)) &&
           contains(Vector3(max.x, max.y, min.z)) &&
           contains(Vector3(max.x, max.y, max.z));
}

Sphere Sphere::merged(const Sphere& other) const {
    Vector3 toOther = other.center - center;
    float dist = toOther.length();
    
    if (dist + other.radius <= radius) {
        return *this; // 当前球体包含另一个球体
    }
    
    if (dist + radius <= other.radius) {
        return other; // 另一个球体包含当前球体
    }
    
    Vector3 newCenter = center + toOther * (radius / (radius + other.radius));
    float newRadius = (radius + dist + other.radius) * 0.5f;
    
    return Sphere(newCenter, newRadius);
}

Sphere& Sphere::merge(const Sphere& other) {
    *this = merged(other);
    return *this;
}

Sphere Sphere::transformed(const Matrix4x4& transform) const {
    Vector3 newCenter = transform.transformPoint(center);
    
    // 计算变换后的最大半径
    Vector3 axes[3] = {
        Vector3(1.0f, 0.0f, 0.0f),
        Vector3(0.0f, 1.0f, 0.0f),
        Vector3(0.0f, 0.0f, 1.0f)
    };
    
    float maxRadius = 0.0f;
    for (int i = 0; i < 3; ++i) {
        Vector3 transformedAxis = transform.transformVector(axes[i]);
        float axisLength = transformedAxis.length();
        if (axisLength > maxRadius) {
            maxRadius = axisLength;
        }
    }
    
    return Sphere(newCenter, radius * maxRadius);
}

// 碰撞检测方法
bool Sphere::intersects(const Ray& ray, float& t) const {
    // 射线与球体相交检测
    // 球体方程：(p - c)·(p - c) = r²
    // 射线方程：p = o + t*d
    
    Vector3 oc = ray.origin - center;
    float a = Vector3::dot(ray.direction, ray.direction); // 射线方向是归一化的，所以a = 1.0f
    float b = 2.0f * Vector3::dot(oc, ray.direction);
    float c = Vector3::dot(oc, oc) - radius * radius;
    
    // 计算判别式
    float discriminant = b * b - 4.0f * a * c;
    
    if (discriminant < 0.0f) {
        // 没有实根，不相交
        return false;
    }
    
    // 计算两个根
    float sqrtDiscriminant = std::sqrt(discriminant);
    float t0 = (-b - sqrtDiscriminant) / (2.0f * a);
    float t1 = (-b + sqrtDiscriminant) / (2.0f * a);
    
    // 找到最小的正根
    if (t0 > Math::EPSILON) {
        t = t0;
        return true;
    }
    
    if (t1 > Math::EPSILON) {
        t = t1;
        return true;
    }
    
    // 没有正根，射线在球体内部或背后
    return false;
}

bool Sphere::intersects(const Vector3& point) const {
    return Math::approximatelyEqual((point - center).lengthSquared(), radius * radius);
}

bool Sphere::intersects(const Sphere& other) const {
    float distSq = (center - other.center).lengthSquared();
    float radiusSum = radius + other.radius;
    return distSq <= radiusSum * radiusSum;
}

bool Sphere::intersects(const Plane& plane) const {
    return plane.intersects(*this);
}

bool Sphere::intersects(const Bounds& bounds) const {
    Vector3 closest = bounds.closestPoint(center);
    return (closest - center).lengthSquared() <= radius * radius;
}

// 静态方法
Sphere Sphere::fromPoints(const Vector3* points, size_t count) {
    return Sphere(points, count);
}

Sphere Sphere::boundingSphere(const Bounds& bounds) {
    return Sphere(bounds);
}

} // namespace AstraAlgebra