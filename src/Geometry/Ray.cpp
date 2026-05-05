// Ray.cpp - 射线类实现，碰撞检测那些

#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Triangle.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

namespace AstraAlgebra {

// 构造函数
Ray::Ray() : origin(Vector3::zero), direction(Vector3::forward) {}

Ray::Ray(const Vector3& origin, const Vector3& direction) 
    : origin(origin), direction(direction.normalized()) {}

Ray::Ray(const Ray& other) : origin(other.origin), direction(other.direction) {}

Ray::Ray(Ray&& other) noexcept : origin(std::move(other.origin)), direction(std::move(other.direction)) {}

// 赋值操作符
Ray& Ray::operator=(const Ray& other) {
    if (this != &other) {
        origin = other.origin;
        direction = other.direction;
    }
    return *this;
}

Ray& Ray::operator=(Ray&& other) noexcept {
    if (this != &other) {
        origin = std::move(other.origin);
        direction = std::move(other.direction);
    }
    return *this;
}

// 射线操作方法
Vector3 Ray::pointAt(float t) const {
    return origin + direction * t;
}

// 射线操作方法（别名，兼容现有接口）
Vector3 Ray::at(float t) const {
    return pointAt(t);
}

// 相交检测方法实现

// 射线与球体相交检测
bool Ray::intersects(const Sphere& sphere, float& t) const {
    // 射线方程：p(t) = origin + t * direction
    // 球体方程：(p - center)·(p - center) = radius²
    
    Vector3 oc = origin - sphere.center;
    float a = Vector3::dot(direction, direction); // 由于direction已归一化，a=1.0f
    float b = 2.0f * Vector3::dot(direction, oc);
    float c = Vector3::dot(oc, oc) - (sphere.radius * sphere.radius);
    
    // 计算判别式
    float discriminant = b * b - 4.0f * a * c;
    
    // 如果判别式小于0，没有交点
    if (discriminant < 0.0f) {
        return false;
    }
    
    // 计算两个交点的t值
    float sqrtDiscriminant = std::sqrt(discriminant);
    float t0 = (-b - sqrtDiscriminant) / (2.0f * a);
    float t1 = (-b + sqrtDiscriminant) / (2.0f * a);
    
    // 找到最小的正t值
    if (t0 > Math::EPSILON) {
        t = t0;
        return true;
    }
    
    if (t1 > Math::EPSILON) {
        t = t1;
        return true;
    }
    
    // 所有交点都在射线起点后面，返回false
    return false;
}

// 射线与平面相交检测
bool Ray::intersects(const Plane& plane, float& t) const {
    // 射线与平面相交检测实现
    // 平面方程：ax + by + cz + d = 0
    // 射线方程：r(t) = origin + t * direction
    
    float denom = plane.a * direction.x + plane.b * direction.y + plane.c * direction.z;
    
    // 如果射线与平面平行，不相交
    if (Math::approximatelyEqual(denom, 0.0f)) {
        return false;
    }
    
    // 计算交点参数t
    t = -(plane.a * origin.x + plane.b * origin.y + plane.c * origin.z + plane.d) / denom;
    
    // 只返回t > 0的交点（射线方向上的交点）
    return t > Math::EPSILON;
}

// 射线与三角形相交检测（使用Möller-Trumbore算法）
bool Ray::intersects(const Triangle& triangle, float& t) const {
    // Möller-Trumbore算法实现
    const Vector3& v0 = triangle.a;
    const Vector3& v1 = triangle.b;
    const Vector3& v2 = triangle.c;
    
    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    
    // 计算叉积：direction × edge2
    Vector3 h = Vector3::cross(direction, edge2);
    float a = Vector3::dot(edge1, h);
    
    // 如果a接近0，射线与三角形平行或在三角形平面内
    if (Math::approximatelyEqual(a, 0.0f)) {
        return false;
    }
    
    float f = 1.0f / a;
    Vector3 s = origin - v0;
    float u = f * Vector3::dot(s, h);
    
    // u不在[0, 1]范围内，交点不在三角形内
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    Vector3 q = Vector3::cross(s, edge1);
    float v = f * Vector3::dot(direction, q);
    
    // v不在[0, 1]范围内，或u + v > 1，交点不在三角形内
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    // 计算t值
    t = f * Vector3::dot(edge2, q);
    
    // 只返回t > 0的交点
    return t > Math::EPSILON;
}

// 射线与AABB相交检测（使用Slab方法）
bool Ray::intersects(const Bounds& bounds, float& t) const {
    // Slab方法实现
    float tMin = -std::numeric_limits<float>::infinity();
    float tMax = std::numeric_limits<float>::infinity();
    
    // 检查X轴方向
    if (!Math::approximatelyEqual(direction.x, 0.0f)) {
        float invD = 1.0f / direction.x;
        float t1 = (bounds.min.x - origin.x) * invD;
        float t2 = (bounds.max.x - origin.x) * invD;
        
        tMin = std::max(tMin, std::min(t1, t2));
        tMax = std::min(tMax, std::max(t1, t2));
    } else {
        // 射线方向平行于X轴
        if (origin.x < bounds.min.x || origin.x > bounds.max.x) {
            return false;
        }
    }
    
    // 检查Y轴方向
    if (!Math::approximatelyEqual(direction.y, 0.0f)) {
        float invD = 1.0f / direction.y;
        float t1 = (bounds.min.y - origin.y) * invD;
        float t2 = (bounds.max.y - origin.y) * invD;
        
        tMin = std::max(tMin, std::min(t1, t2));
        tMax = std::min(tMax, std::max(t1, t2));
    } else {
        // 射线方向平行于Y轴
        if (origin.y < bounds.min.y || origin.y > bounds.max.y) {
            return false;
        }
    }
    
    // 检查Z轴方向
    if (!Math::approximatelyEqual(direction.z, 0.0f)) {
        float invD = 1.0f / direction.z;
        float t1 = (bounds.min.z - origin.z) * invD;
        float t2 = (bounds.max.z - origin.z) * invD;
        
        tMin = std::max(tMin, std::min(t1, t2));
        tMax = std::min(tMax, std::max(t1, t2));
    } else {
        // 射线方向平行于Z轴
        if (origin.z < bounds.min.z || origin.z > bounds.max.z) {
            return false;
        }
    }
    
    // 检查是否有交点
    if (tMax < tMin || tMax < Math::EPSILON) {
        return false;
    }
    
    // 计算最小的正t值
    t = tMin;
    if (t < Math::EPSILON) {
        t = tMax;
    }
    
    return t > Math::EPSILON;
}

// 便利方法：直接返回相交距离
float Ray::intersect(const Sphere& sphere) const {
    float t;
    if (intersects(sphere, t)) {
        return t;
    }
    return -1.0f;
}

float Ray::intersect(const Plane& plane) const {
    float t;
    if (intersects(plane, t)) {
        return t;
    }
    return -1.0f;
}

float Ray::intersect(const Triangle& triangle) const {
    float t;
    if (intersects(triangle, t)) {
        return t;
    }
    return -1.0f;
}

float Ray::intersect(const Bounds& bounds) const {
    float t;
    if (intersects(bounds, t)) {
        return t;
    }
    return -1.0f;
}

} // namespace AstraAlgebra