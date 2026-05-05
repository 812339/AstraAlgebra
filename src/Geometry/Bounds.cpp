// Bounds.cpp - AABB包围盒类实现，碰撞检测用得很多

#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <algorithm>

namespace AstraAlgebra {

// 构造函数
Bounds::Bounds() : min(std::numeric_limits<float>::max()), max(std::numeric_limits<float>::lowest()) {
}

Bounds::Bounds(const Vector3& min, const Vector3& max) : min(min), max(max) {
}

Bounds::Bounds(const Vector3& point) : min(point), max(point) {
}

Bounds::Bounds(const Bounds& other) : min(other.min), max(other.max) {
}

Bounds::Bounds(Bounds&& other) noexcept : min(std::move(other.min)), max(std::move(other.max)) {
    other.min = Vector3(std::numeric_limits<float>::max());
    other.max = Vector3(std::numeric_limits<float>::lowest());
}

Bounds::Bounds(const Vector3* points, size_t count) {
    if (count == 0) {
        *this = Bounds();
        return;
    }
    
    min = points[0];
    max = points[0];
    
    for (size_t i = 1; i < count; ++i) {
        if (points[i].x < min.x) min.x = points[i].x;
        if (points[i].y < min.y) min.y = points[i].y;
        if (points[i].z < min.z) min.z = points[i].z;
        if (points[i].x > max.x) max.x = points[i].x;
        if (points[i].y > max.y) max.y = points[i].y;
        if (points[i].z > max.z) max.z = points[i].z;
    }
}

Bounds::Bounds(const Sphere& sphere) {
    min = sphere.center - Vector3(sphere.radius, sphere.radius, sphere.radius);
    max = sphere.center + Vector3(sphere.radius, sphere.radius, sphere.radius);
}

// 赋值操作符
Bounds& Bounds::operator=(const Bounds& other) {
    if (this != &other) {
        min = other.min;
        max = other.max;
    }
    return *this;
}

Bounds& Bounds::operator=(Bounds&& other) noexcept {
    if (this != &other) {
        min = std::move(other.min);
        max = std::move(other.max);
        
        other.min = Vector3(std::numeric_limits<float>::max());
        other.max = Vector3(std::numeric_limits<float>::lowest());
    }
    return *this;
}

// 边界框方法
Vector3 Bounds::getCenter() const {
    return (min + max) * 0.5f;
}

Vector3 Bounds::getExtents() const {
    return (max - min) * 0.5f;
}

Vector3 Bounds::getSize() const {
    return max - min;
}

float Bounds::volume() const {
    Vector3 size = getSize();
    return size.x * size.y * size.z;
}

float Bounds::surfaceArea() const {
    Vector3 size = getSize();
    return 2.0f * (size.x * size.y + size.x * size.z + size.y * size.z);
}

bool Bounds::isValid() const {
    return min.x <= max.x && min.y <= max.y && min.z <= max.z;
}

void Bounds::invalidate() {
    min = Vector3(std::numeric_limits<float>::max());
    max = Vector3(std::numeric_limits<float>::lowest());
}

bool Bounds::contains(const Vector3& point) const {
    return point.x >= min.x && point.x <= max.x &&
           point.y >= min.y && point.y <= max.y &&
           point.z >= min.z && point.z <= max.z;
}

bool Bounds::contains(const Bounds& other) const {
    return other.min.x >= min.x && other.max.x <= max.x &&
           other.min.y >= min.y && other.max.y <= max.y &&
           other.min.z >= min.z && other.max.z <= max.z;
}

bool Bounds::contains(const Sphere& sphere) const {
    return contains(sphere.center - Vector3(sphere.radius, sphere.radius, sphere.radius)) &&
           contains(sphere.center + Vector3(sphere.radius, sphere.radius, sphere.radius));
}

Bounds Bounds::merged(const Bounds& other) const {
    if (!isValid()) return other;
    if (!other.isValid()) return *this;
    
    return Bounds(
        Vector3(
            std::min(min.x, other.min.x),
            std::min(min.y, other.min.y),
            std::min(min.z, other.min.z)
        ),
        Vector3(
            std::max(max.x, other.max.x),
            std::max(max.y, other.max.y),
            std::max(max.z, other.max.z)
        )
    );
}

Bounds& Bounds::merge(const Bounds& other) {
    *this = merged(other);
    return *this;
}

Bounds Bounds::merged(const Vector3& point) const {
    if (!isValid()) {
        return Bounds(point, point);
    }
    
    return Bounds(
        Vector3(
            std::min(min.x, point.x),
            std::min(min.y, point.y),
            std::min(min.z, point.z)
        ),
        Vector3(
            std::max(max.x, point.x),
            std::max(max.y, point.y),
            std::max(max.z, point.z)
        )
    );
}

Bounds& Bounds::merge(const Vector3& point) {
    *this = merged(point);
    return *this;
}

Vector3 Bounds::closestPoint(const Vector3& point) const {
    return Vector3(
        std::max(min.x, std::min(max.x, point.x)),
        std::max(min.y, std::min(max.y, point.y)),
        std::max(min.z, std::min(max.z, point.z))
    );
}

Bounds Bounds::transformed(const Matrix4x4& transform) const {
    if (!isValid()) {
        return *this;
    }
    
    // 变换所有八个顶点，然后创建新的包围盒
    Vector3 points[8] = {
        transform.transformPoint(Vector3(min.x, min.y, min.z)),
        transform.transformPoint(Vector3(min.x, min.y, max.z)),
        transform.transformPoint(Vector3(min.x, max.y, min.z)),
        transform.transformPoint(Vector3(min.x, max.y, max.z)),
        transform.transformPoint(Vector3(max.x, min.y, min.z)),
        transform.transformPoint(Vector3(max.x, min.y, max.z)),
        transform.transformPoint(Vector3(max.x, max.y, min.z)),
        transform.transformPoint(Vector3(max.x, max.y, max.z))
    };
    
    return Bounds(points, 8);
}

// 碰撞检测方法
bool Bounds::intersects(const Ray& ray, float& t) const {
    // 射线与AABB相交检测，使用Slab方法
    float tMin = std::numeric_limits<float>::lowest();
    float tMax = std::numeric_limits<float>::max();
    
    // 处理x轴
    float t1 = (min.x - ray.origin.x) / ray.direction.x;
    float t2 = (max.x - ray.origin.x) / ray.direction.x;
    tMin = std::max(tMin, std::min(t1, t2));
    tMax = std::min(tMax, std::max(t1, t2));
    
    if (tMin > tMax) {
        return false;
    }
    
    // 处理y轴
    t1 = (min.y - ray.origin.y) / ray.direction.y;
    t2 = (max.y - ray.origin.y) / ray.direction.y;
    tMin = std::max(tMin, std::min(t1, t2));
    tMax = std::min(tMax, std::max(t1, t2));
    
    if (tMin > tMax) {
        return false;
    }
    
    // 处理z轴
    t1 = (min.z - ray.origin.z) / ray.direction.z;
    t2 = (max.z - ray.origin.z) / ray.direction.z;
    tMin = std::max(tMin, std::min(t1, t2));
    tMax = std::min(tMax, std::max(t1, t2));
    
    // 检查是否相交
    if (tMin > tMax) {
        return false;
    }
    
    // 只返回t > 0的交点（射线方向上的交点）
    if (tMin > Math::EPSILON) {
        t = tMin;
        return true;
    }
    
    if (tMax > Math::EPSILON) {
        t = tMax;
        return true;
    }
    
    return false;
}

bool Bounds::intersects(const Bounds& other) const {
    return min.x <= other.max.x && max.x >= other.min.x &&
           min.y <= other.max.y && max.y >= other.min.y &&
           min.z <= other.max.z && max.z >= other.min.z;
}

bool Bounds::intersects(const Sphere& sphere) const {
    return sphere.intersects(*this);
}

bool Bounds::intersects(const Plane& plane) const {
    return plane.intersects(*this);
}

// 静态方法
Bounds Bounds::fromPoints(const Vector3* points, size_t count) {
    return Bounds(points, count);
}

Bounds Bounds::fromSphere(const Sphere& sphere) {
    return Bounds(sphere);
}

Bounds Bounds::infinity() {
    return Bounds(
        Vector3(std::numeric_limits<float>::lowest()),
        Vector3(std::numeric_limits<float>::max())
    );
}

Bounds Bounds::zero() {
    return Bounds(Vector3::zero, Vector3::zero);
}

} // namespace AstraAlgebra