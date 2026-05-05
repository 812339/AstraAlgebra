// Cone.cpp - 圆锥类实现
// 顶点加底面，碰撞检测用

#include <AstraAlgebra/Geometry/Cone.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

namespace AstraAlgebra {

// 默认构造函数
Cone::Cone() : apex(0.0f, 1.0f, 0.0f), baseCenter(0.0f, 0.0f, 0.0f), radius(0.5f) {}

// 使用顶点、底面中心和半径构造
Cone::Cone(const Vector3& apex, const Vector3& baseCenter, float radius) : apex(apex), baseCenter(baseCenter), radius(radius) {}

// 拷贝构造函数
Cone::Cone(const Cone& other) : apex(other.apex), baseCenter(other.baseCenter), radius(other.radius) {}

// 移动构造函数
Cone::Cone(Cone&& other) noexcept : apex(std::move(other.apex)), baseCenter(std::move(other.baseCenter)), radius(other.radius) {}

// 拷贝赋值
Cone& Cone::operator=(const Cone& other) {
    if (this != &other) {
        apex = other.apex;
        baseCenter = other.baseCenter;
        radius = other.radius;
    }
    return *this;
}

// 移动赋值
Cone& Cone::operator=(Cone&& other) noexcept {
    if (this != &other) {
        apex = std::move(other.apex);
        baseCenter = std::move(other.baseCenter);
        radius = other.radius;
    }
    return *this;
}

// 获取圆锥高度
float Cone::getHeight() const {
    return (baseCenter - apex).length();
}

// 获取圆锥轴线向量
Vector3 Cone::getAxis() const {
    return baseCenter - apex;
}

// 获取圆锥斜高
float Cone::getSlantHeight() const {
    float height = getHeight();
    return std::sqrt(height * height + radius * radius);
}

// 获取圆锥半顶角（弧度）
float Cone::getAngle() const {
    float height = getHeight();
    return std::atan(radius / height);
}

// 获取圆锥包围球
Sphere Cone::getSphere() const {
    Vector3 center = (apex + baseCenter) * 0.5f;
    float height = getHeight();
    float sphereRadius = std::max(getSlantHeight() * 0.5f, (apex - center).length());
    return Sphere(center, sphereRadius);
}

// 获取圆锥包围盒
Bounds Cone::getBounds() const {
    Vector3 minPoint = Vector3::min(apex, baseCenter);
    Vector3 maxPoint = Vector3::max(apex, baseCenter);
    
    // 扩展包围盒以包含圆锥底面
    float radiusXZ = radius;
    minPoint.x -= radiusXZ;
    minPoint.z -= radiusXZ;
    maxPoint.x += radiusXZ;
    maxPoint.z += radiusXZ;
    
    return Bounds(minPoint, maxPoint);
}

// 获取圆锥上距离给定点最近的点
Vector3 Cone::closestPoint(const Vector3& point) const {
    Vector3 axis = getAxis();
    float height = getHeight();
    float heightSquared = height * height;
    
    // 如果高度为0，返回顶点
    if (Math::approximatelyEqual(heightSquared, 0.0f)) {
        Vector3 direction = point - apex;
        float distance = direction.length();
        if (distance <= radius) {
            return point;
        }
        return apex + direction.normalized() * radius;
    }
    
    Vector3 apexToPoint = point - apex;
    float t = Vector3::dot(apexToPoint, axis) / heightSquared;
    t = Math::clamp(t, 0.0f, 1.0f);
    
    // 计算圆锥表面上的点
    float radiusAtT = radius * (1.0f - t);
    Vector3 coneSurfacePoint = apex + axis * t;
    
    Vector3 pointToSurface = point - coneSurfacePoint;
    Vector3 coneRadiusDirection = pointToSurface - Vector3::dot(pointToSurface, axis.normalized()) * axis.normalized();
    float coneRadiusDirectionLength = coneRadiusDirection.length();
    
    // 如果点在圆锥内部，返回点到圆锥表面的投影
    if (coneRadiusDirectionLength <= radiusAtT) {
        return point;
    }
    
    // 否则返回圆锥表面上的点
    if (coneRadiusDirectionLength > 0.0f) {
        coneRadiusDirection = coneRadiusDirection.normalized();
    } else {
        coneRadiusDirection = Vector3(1.0f, 0.0f, 0.0f);
    }
    
    return coneSurfacePoint + coneRadiusDirection * radiusAtT;
}

// 计算点到圆锥的距离
float Cone::distanceToPoint(const Vector3& point) const {
    Vector3 closest = closestPoint(point);
    return (closest - point).length();
}

// 与射线相交检测
bool Cone::intersects(const Ray& ray, float& t) const {
    Vector3 axis = getAxis();
    float height = getHeight();
    
    if (Math::approximatelyEqual(height, 0.0f)) {
        return false;
    }
    
    Vector3 v = ray.direction;
    Vector3 p = ray.origin - apex;
    Vector3 a = axis;
    
    float a2 = a.lengthSquared();
    float v2 = v.lengthSquared();
    float p2 = p.lengthSquared();
    float av = Vector3::dot(a, v);
    float ap = Vector3::dot(a, p);
    float vp = Vector3::dot(v, p);
    
    // 圆锥方程：(p + vt)^2 * (a^2 r^2) - (ap + av t)^2 r^2 = 0
    float A = a2 * v2 - av * av;
    float B = 2.0f * (a2 * vp - ap * av);
    float C = a2 * p2 - ap * ap;
    
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
    
    // 检查交点是否在圆锥高度范围内
    Vector3 hitPoint = ray.origin + ray.direction * t;
    Vector3 hitToApex = hitPoint - apex;
    float projection = Vector3::dot(hitToApex, axis) / a2;
    
    if (projection < 0.0f || projection > 1.0f) {
        return false;
    }
    
    return true;
}

// 判断点是否在圆锥内
bool Cone::intersects(const Vector3& point) const {
    float distance = distanceToPoint(point);
    return distance <= Math::EPSILON;
}

// 与球体相交检测
bool Cone::intersects(const Sphere& sphere) const {
    Vector3 closest = closestPoint(sphere.center);
    float distanceSquared = (closest - sphere.center).lengthSquared();
    return distanceSquared <= sphere.radius * sphere.radius;
}

// 与平面相交检测
bool Cone::intersects(const Plane& plane) const {
    // 检查圆锥顶点和底面中心是否在平面两侧
    float distApex = plane.distanceToPoint(apex);
    float distBase = plane.distanceToPoint(baseCenter);
    
    // 如果顶点和底面中心在平面两侧，圆锥与平面相交
    if ((distApex < 0.0f && distBase > 0.0f) || (distApex > 0.0f && distBase < 0.0f)) {
        return true;
    }
    
    // 检查圆锥顶点或底面中心到平面的距离是否小于等于球体半径
    return std::abs(distApex) <= radius || std::abs(distBase) <= radius;
}

// 与AABB相交检测
bool Cone::intersects(const Bounds& bounds) const {
    Vector3 closest = bounds.closestPoint(apex);
    return distanceToPoint(closest) <= Math::EPSILON;
}

// 从顶点、底面中心和半径创建圆锥
Cone Cone::fromPoints(const Vector3& apex, const Vector3& baseCenter, float radius) {
    return Cone(apex, baseCenter, radius);
}

// 创建垂直圆锥（沿Y轴，顶点向上）
Cone Cone::vertical(float height, float radius) {
    Vector3 apex(0.0f, height, 0.0f);
    Vector3 baseCenter(0.0f, 0.0f, 0.0f);
    return Cone(apex, baseCenter, radius);
}

// 创建垂直圆锥（沿Y轴，顶点向下）
Cone Cone::verticalDown(float height, float radius) {
    Vector3 apex(0.0f, 0.0f, 0.0f);
    Vector3 baseCenter(0.0f, height, 0.0f);
    return Cone(apex, baseCenter, radius);
}

// 创建水平圆锥（沿X轴）
Cone Cone::horizontalX(float length, float radius) {
    Vector3 apex(length, 0.0f, 0.0f);
    Vector3 baseCenter(0.0f, 0.0f, 0.0f);
    return Cone(apex, baseCenter, radius);
}

// 创建水平圆锥（沿Z轴）
Cone Cone::horizontalZ(float length, float radius) {
    Vector3 apex(0.0f, 0.0f, length);
    Vector3 baseCenter(0.0f, 0.0f, 0.0f);
    return Cone(apex, baseCenter, radius);
}

} // namespace AstraAlgebra