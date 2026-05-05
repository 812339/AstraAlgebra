// Capsule.cpp - 胶囊体类实现，角色碰撞最爱

#include <AstraAlgebra/Geometry/Capsule.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

namespace AstraAlgebra {

// 默认构造函数
Capsule::Capsule() : point1(0.0f, -0.5f, 0.0f), point2(0.0f, 0.5f, 0.0f), radius(0.5f) {}

// 使用两个端点和半径构造
Capsule::Capsule(const Vector3& point1, const Vector3& point2, float radius) : point1(point1), point2(point2), radius(radius) {}

// 拷贝构造函数
Capsule::Capsule(const Capsule& other) : point1(other.point1), point2(other.point2), radius(other.radius) {}

// 移动构造函数
Capsule::Capsule(Capsule&& other) noexcept : point1(std::move(other.point1)), point2(std::move(other.point2)), radius(other.radius) {}

// 拷贝赋值
Capsule& Capsule::operator=(const Capsule& other) {
    if (this != &other) {
        point1 = other.point1;
        point2 = other.point2;
        radius = other.radius;
    }
    return *this;
}

// 移动赋值
Capsule& Capsule::operator=(Capsule&& other) noexcept {
    if (this != &other) {
        point1 = std::move(other.point1);
        point2 = std::move(other.point2);
        radius = other.radius;
    }
    return *this;
}

// 获取胶囊体线段长度
float Capsule::getLength() const {
    return (point2 - point1).length();
}

// 获取胶囊体轴线向量
Vector3 Capsule::getAxis() const {
    return point2 - point1;
}

// 获取胶囊体包围球
Sphere Capsule::getSphere() const {
    Vector3 center = (point1 + point2) * 0.5f;
    float halfLength = getLength() * 0.5f;
    float sphereRadius = halfLength + radius;
    return Sphere(center, sphereRadius);
}

// 获取胶囊体包围盒
Bounds Capsule::getBounds() const {
    Vector3 minPoint = Vector3::min(point1, point2);
    Vector3 maxPoint = Vector3::max(point1, point2);
    
    // 扩展包围盒以包含胶囊体半径
    minPoint.x -= radius;
    minPoint.y -= radius;
    minPoint.z -= radius;
    maxPoint.x += radius;
    maxPoint.y += radius;
    maxPoint.z += radius;
    
    return Bounds(minPoint, maxPoint);
}

// 获取胶囊体上距离给定点最近的点
Vector3 Capsule::closestPoint(const Vector3& point) const {
    Vector3 axis = getAxis();
    float lengthSquared = axis.lengthSquared();
    
    // 如果线段长度为0，返回点到球体的最近点
    if (Math::approximatelyEqual(lengthSquared, 0.0f)) {
        Vector3 direction = point - point1;
        float distance = direction.length();
        if (distance <= radius) {
            return point;
        }
        return point1 + direction.normalized() * radius;
    }
    
    // 计算点在线段上的投影参数
    float t = Vector3::dot(point - point1, axis) / lengthSquared;
    t = Math::clamp(t, 0.0f, 1.0f);
    
    // 计算线段上的投影点
    Vector3 projectedPoint = point1 + axis * t;
    
    // 计算投影点到给定点的向量
    Vector3 direction = point - projectedPoint;
    float distanceSquared = direction.lengthSquared();
    
    // 如果距离小于等于半径，返回投影点
    if (distanceSquared <= radius * radius) {
        return point;
    }
    
    // 否则返回胶囊体表面上的点
    return projectedPoint + direction.normalized() * radius;
}

// 计算点到胶囊体的距离
float Capsule::distanceToPoint(const Vector3& point) const {
    Vector3 closest = closestPoint(point);
    return (closest - point).length();
}

// 与射线相交检测
bool Capsule::intersects(const Ray& ray, float& t) const {
    // 胶囊体与射线相交检测实现
    // 胶囊体可以看作是线段和球体的组合，所以可以先计算射线与线段的最近点，然后判断距离是否小于等于半径
    Vector3 axis = getAxis();
    Vector3 rayOriginToPoint1 = ray.origin - point1;
    
    float a = Vector3::dot(ray.direction, ray.direction);
    float b = 2.0f * Vector3::dot(ray.direction, rayOriginToPoint1);
    float c = Vector3::dot(rayOriginToPoint1, rayOriginToPoint1) - radius * radius;
    
    // 计算射线与球体的相交
    float discriminant = b * b - 4.0f * a * c;
    if (discriminant < 0.0f) {
        return false;
    }
    
    float sqrtDiscriminant = std::sqrt(discriminant);
    float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
    float t2 = (-b + sqrtDiscriminant) / (2.0f * a);
    
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
    
    // 检查交点是否在胶囊体的有效范围内
    Vector3 hitPoint = ray.origin + ray.direction * t;
    Vector3 closestOnSegment = closestPoint(hitPoint);
    float distanceSquared = (hitPoint - closestOnSegment).lengthSquared();
    
    return distanceSquared <= radius * radius;
}

// 判断点是否在胶囊体内
bool Capsule::intersects(const Vector3& point) const {
    float distance = distanceToPoint(point);
    return distance <= Math::EPSILON;
}

// 与球体相交检测
bool Capsule::intersects(const Sphere& sphere) const {
    Vector3 closest = closestPoint(sphere.center);
    float distanceSquared = (closest - sphere.center).lengthSquared();
    float radiusSum = radius + sphere.radius;
    return distanceSquared <= radiusSum * radiusSum;
}

// 与平面相交检测
bool Capsule::intersects(const Plane& plane) const {
    // 检查胶囊体的两个端点是否在平面两侧
    float dist1 = plane.distanceToPoint(point1);
    float dist2 = plane.distanceToPoint(point2);
    
    // 如果两个端点在平面两侧，胶囊体与平面相交
    if ((dist1 < 0.0f && dist2 > 0.0f) || (dist1 > 0.0f && dist2 < 0.0f)) {
        return true;
    }
    
    // 检查两个端点到平面的距离是否小于等于半径
    return std::abs(dist1) <= radius || std::abs(dist2) <= radius;
}

// 与AABB相交检测
bool Capsule::intersects(const Bounds& bounds) const {
    Vector3 closest = bounds.closestPoint((point1 + point2) * 0.5f);
    return distanceToPoint(closest) <= radius;
}

// 从两个端点和半径创建胶囊体
Capsule Capsule::fromPoints(const Vector3& point1, const Vector3& point2, float radius) {
    return Capsule(point1, point2, radius);
}

// 创建垂直胶囊体（沿Y轴）
Capsule Capsule::vertical(float height, float radius) {
    float halfHeight = height * 0.5f;
    Vector3 point1(0.0f, -halfHeight, 0.0f);
    Vector3 point2(0.0f, halfHeight, 0.0f);
    return Capsule(point1, point2, radius);
}

// 创建水平胶囊体（沿X轴）
Capsule Capsule::horizontalX(float length, float radius) {
    float halfLength = length * 0.5f;
    Vector3 point1(-halfLength, 0.0f, 0.0f);
    Vector3 point2(halfLength, 0.0f, 0.0f);
    return Capsule(point1, point2, radius);
}

// 创建水平胶囊体（沿Z轴）
Capsule Capsule::horizontalZ(float length, float radius) {
    float halfLength = length * 0.5f;
    Vector3 point1(0.0f, 0.0f, -halfLength);
    Vector3 point2(0.0f, 0.0f, halfLength);
    return Capsule(point1, point2, radius);
}

} // namespace AstraAlgebra