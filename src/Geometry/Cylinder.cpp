// Cylinder.cpp - 圆柱类实现

#include <AstraAlgebra/Geometry/Cylinder.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

namespace AstraAlgebra {

// 默认构造函数
Cylinder::Cylinder() : center1(0.0f, -0.5f, 0.0f), center2(0.0f, 0.5f, 0.0f), radius(0.5f) {}

// 使用两个中心点和半径构造
Cylinder::Cylinder(const Vector3& center1, const Vector3& center2, float radius) : center1(center1), center2(center2), radius(radius) {}

// 拷贝构造函数
Cylinder::Cylinder(const Cylinder& other) : center1(other.center1), center2(other.center2), radius(other.radius) {}

// 移动构造函数
Cylinder::Cylinder(Cylinder&& other) noexcept : center1(std::move(other.center1)), center2(std::move(other.center2)), radius(other.radius) {}

// 拷贝赋值
Cylinder& Cylinder::operator=(const Cylinder& other) {
    if (this != &other) {
        center1 = other.center1;
        center2 = other.center2;
        radius = other.radius;
    }
    return *this;
}

// 移动赋值
Cylinder& Cylinder::operator=(Cylinder&& other) noexcept {
    if (this != &other) {
        center1 = std::move(other.center1);
        center2 = std::move(other.center2);
        radius = other.radius;
    }
    return *this;
}

// 获取圆柱高度
float Cylinder::getHeight() const {
    return (center2 - center1).length();
}

// 获取圆柱轴线向量
Vector3 Cylinder::getAxis() const {
    return center2 - center1;
}

// 获取圆柱包围球
Sphere Cylinder::getSphere() const {
    Vector3 center = (center1 + center2) * 0.5f;
    float height = getHeight();
    float sphereRadius = std::sqrt((height * 0.5f) * (height * 0.5f) + radius * radius);
    return Sphere(center, sphereRadius);
}

// 获取圆柱包围盒
Bounds Cylinder::getBounds() const {
    Vector3 minPoint = Vector3::min(center1, center2);
    Vector3 maxPoint = Vector3::max(center1, center2);
    
    minPoint.x -= radius;
    minPoint.y -= radius;
    minPoint.z -= radius;
    
    maxPoint.x += radius;
    maxPoint.y += radius;
    maxPoint.z += radius;
    
    return Bounds(minPoint, maxPoint);
}

// 获取圆柱上距离给定点最近的点
Vector3 Cylinder::closestPoint(const Vector3& point) const {
    // 计算线段方向向量
    Vector3 axis = getAxis();
    float lengthSquared = axis.lengthSquared();
    
    // 如果线段长度为0，返回圆心
    if (Math::approximatelyEqual(lengthSquared, 0.0f)) {
        Vector3 direction = point - center1;
        float distance = direction.length();
        if (distance <= radius) {
            return point;
        }
        return center1 + direction.normalized() * radius;
    }
    
    // 计算点在线段上的投影参数
    float t = Vector3::dot(point - center1, axis) / lengthSquared;
    t = Math::clamp(t, 0.0f, 1.0f);
    
    // 计算投影点
    Vector3 projectedPoint = center1 + axis * t;
    
    // 计算投影点到给定点的向量
    Vector3 direction = point - projectedPoint;
    float distanceSquared = direction.lengthSquared();
    
    // 如果距离小于等于半径，返回投影点
    if (distanceSquared <= radius * radius) {
        return point;
    }
    
    // 否则返回圆柱表面上的点
    return projectedPoint + direction.normalized() * radius;
}

// 计算点到圆柱的距离
float Cylinder::distanceToPoint(const Vector3& point) const {
    Vector3 closest = closestPoint(point);
    return (closest - point).length();
}

// 与射线相交检测
bool Cylinder::intersects(const Ray& ray, float& t) const {
    // 圆柱与射线相交检测实现
    // 这是一个简化的实现，更复杂的实现需要考虑射线与圆柱侧面和底面的相交
    // 这里仅实现了射线与圆柱侧面的相交
    Vector3 axis = getAxis();
    float halfHeight = getHeight() * 0.5f;
    Vector3 center = (center1 + center2) * 0.5f;
    
    Vector3 w = ray.origin - center;
    Vector3 aCrossW = Vector3::cross(axis, w);
    Vector3 aCrossD = Vector3::cross(axis, ray.direction);
    
    float a2 = axis.lengthSquared();
    float d2 = ray.direction.lengthSquared();
    float ad = Vector3::dot(axis, ray.direction);
    float aw = Vector3::dot(axis, w);
    float aCrossW2 = aCrossW.lengthSquared();
    
    // 求解二次方程：t^2*(a2*d2 - ad^2) + 2*t*(a2*Vector3::dot(w, ray.direction) - ad*aw) + (a2*w.lengthSquared() - aw^2 - a2*radius^2) = 0
    float A = a2 * d2 - ad * ad;
    float B = 2.0f * (a2 * Vector3::dot(w, ray.direction) - ad * aw);
    float C = a2 * (w.lengthSquared() - radius * radius) - aCrossW2;
    
    float discriminant = B * B - 4.0f * A * C;
    if (discriminant < 0.0f) {
        return false;
    }
    
    float sqrtDiscriminant = std::sqrt(discriminant);
    float t1 = (-B - sqrtDiscriminant) / (2.0f * A);
    float t2 = (-B + sqrtDiscriminant) / (2.0f * A);
    
    // 寻找最小的正t值
    float tMin = std::min(t1, t2);
    float tMax = std::max(t1, t2);
    
    if (tMin > 0.0f) {
        t = tMin;
    } else if (tMax > 0.0f) {
        t = tMax;
    } else {
        return false;
    }
    
    // 检查交点是否在圆柱高度范围内
    Vector3 hitPoint = ray.origin + ray.direction * t;
    Vector3 hitPointToCenter = hitPoint - center;
    float projection = Vector3::dot(hitPointToCenter, axis) / a2;
    
    if (std::abs(projection) > halfHeight) {
        return false;
    }
    
    return true;
}

// 判断点是否在圆柱内
bool Cylinder::intersects(const Vector3& point) const {
    float distance = distanceToPoint(point);
    return distance <= Math::EPSILON;
}

// 与球体相交检测
bool Cylinder::intersects(const Sphere& sphere) const {
    Vector3 closest = closestPoint(sphere.center);
    float distanceSquared = (closest - sphere.center).lengthSquared();
    float radiusSum = radius + sphere.radius;
    return distanceSquared <= radiusSum * radiusSum;
}

// 与平面相交检测
bool Cylinder::intersects(const Plane& plane) const {
    // 检查圆柱的两个底面中心是否在平面两侧
    float dist1 = plane.distanceToPoint(center1);
    float dist2 = plane.distanceToPoint(center2);
    
    // 如果两个底面中心在平面两侧，圆柱与平面相交
    if ((dist1 < 0.0f && dist2 > 0.0f) || (dist1 > 0.0f && dist2 < 0.0f)) {
        return true;
    }
    
    // 检查圆柱的两个底面中心到平面的距离是否小于等于半径
    return std::abs(dist1) <= radius || std::abs(dist2) <= radius;
}

// 与AABB相交检测
bool Cylinder::intersects(const Bounds& bounds) const {
    Vector3 closest = bounds.closestPoint((center1 + center2) * 0.5f);
    return distanceToPoint(closest) <= radius;
}

// 从两个中心点和半径创建圆柱
Cylinder Cylinder::fromPoints(const Vector3& center1, const Vector3& center2, float radius) {
    return Cylinder(center1, center2, radius);
}

// 创建垂直圆柱（沿Y轴）
Cylinder Cylinder::vertical(float height, float radius) {
    float halfHeight = height * 0.5f;
    Vector3 center1(0.0f, -halfHeight, 0.0f);
    Vector3 center2(0.0f, halfHeight, 0.0f);
    return Cylinder(center1, center2, radius);
}

// 创建水平圆柱（沿X轴）
Cylinder Cylinder::horizontalX(float length, float radius) {
    float halfLength = length * 0.5f;
    Vector3 center1(-halfLength, 0.0f, 0.0f);
    Vector3 center2(halfLength, 0.0f, 0.0f);
    return Cylinder(center1, center2, radius);
}

// 创建水平圆柱（沿Z轴）
Cylinder Cylinder::horizontalZ(float length, float radius) {
    float halfLength = length * 0.5f;
    Vector3 center1(0.0f, 0.0f, -halfLength);
    Vector3 center2(0.0f, 0.0f, halfLength);
    return Cylinder(center1, center2, radius);
}

} // namespace AstraAlgebra