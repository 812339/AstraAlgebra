// Triangle.cpp - 三角形类实现，3D建模和碰撞检测都离不开

#include <AstraAlgebra/Geometry/Triangle.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

namespace AstraAlgebra {

// 构造函数
Triangle::Triangle() : a(Vector3::zero), b(Vector3::zero), c(Vector3::zero) {
}

Triangle::Triangle(const Vector3& a, const Vector3& b, const Vector3& c) : a(a), b(b), c(c) {
}

Triangle::Triangle(const Triangle& other) : a(other.a), b(other.b), c(other.c) {
}

Triangle::Triangle(Triangle&& other) noexcept : a(std::move(other.a)), b(std::move(other.b)), c(std::move(other.c)) {
}

// 赋值操作符
Triangle& Triangle::operator=(const Triangle& other) {
    if (this != &other) {
        a = other.a;
        b = other.b;
        c = other.c;
    }
    return *this;
}

Triangle& Triangle::operator=(Triangle&& other) noexcept {
    if (this != &other) {
        a = std::move(other.a);
        b = std::move(other.b);
        c = std::move(other.c);
    }
    return *this;
}

// 三角面方法
Vector3 Triangle::getNormal() const {
    Vector3 edge1 = b - a;
    Vector3 edge2 = c - a;
    return Vector3::cross(edge1, edge2).normalized();
}

float Triangle::getArea() const {
    Vector3 edge1 = b - a;
    Vector3 edge2 = c - a;
    return Vector3::cross(edge1, edge2).length() * 0.5f;
}

Bounds Triangle::getBounds() const {
    Vector3 minVec = Vector3(
        std::min(a.x, std::min(b.x, c.x)),
        std::min(a.y, std::min(b.y, c.y)),
        std::min(a.z, std::min(b.z, c.z))
    );
    
    Vector3 maxVec = Vector3(
        std::max(a.x, std::max(b.x, c.x)),
        std::max(a.y, std::max(b.y, c.y)),
        std::max(a.z, std::max(b.z, c.z))
    );
    
    return Bounds(minVec, maxVec);
}

Sphere Triangle::getSphere() const {
    Vector3 center = (a + b + c) / 3.0f;
    float radiusSq = std::max(
        (a - center).lengthSquared(),
        std::max((b - center).lengthSquared(), (c - center).lengthSquared())
    );
    return Sphere(center, std::sqrt(radiusSq));
}

Plane Triangle::getPlane() const {
    return Plane(getNormal(), a);
}

// 点与三角面关系
bool Triangle::contains(const Vector3& point) const {
    // 使用重心坐标判断点是否在三角面内
    Vector3 edge1 = b - a;
    Vector3 edge2 = c - a;
    Vector3 vp = point - a;
    
    // 计算行列式
    float d00 = Vector3::dot(edge1, edge1);
    float d01 = Vector3::dot(edge1, edge2);
    float d11 = Vector3::dot(edge2, edge2);
    float d20 = Vector3::dot(vp, edge1);
    float d21 = Vector3::dot(vp, edge2);
    
    float denominator = d00 * d11 - d01 * d01;
    if (Math::approximatelyEqual(denominator, 0.0f)) {
        return false;
    }
    
    float invDenominator = 1.0f / denominator;
    float v = (d11 * d20 - d01 * d21) * invDenominator;
    float w = (d00 * d21 - d01 * d20) * invDenominator;
    float u = 1.0f - v - w;
    
    // 检查重心坐标是否都在[0, 1]范围内
    return u >= -Math::EPSILON && v >= -Math::EPSILON && w >= -Math::EPSILON && u + v + w <= 1.0f + Math::EPSILON;
}

Vector3 Triangle::closestPoint(const Vector3& point) const {
    // 计算点到三角面的最近点
    Plane plane = getPlane();
    Vector3 projected = plane.projectPoint(point);
    
    // 如果投影点在三角面内，直接返回
    if (contains(projected)) {
        return projected;
    }
    
    // 否则，找到最近的边或顶点
    Vector3 closest = a;
    float minDistanceSq = (point - a).lengthSquared();
    
    // 检查边ab
    Vector3 edgeAB = b - a;
    float t = Math::clamp(Vector3::dot(point - a, edgeAB) / edgeAB.lengthSquared(), 0.0f, 1.0f);
    Vector3 pointAB = a + edgeAB * t;
    float distanceABSq = (point - pointAB).lengthSquared();
    if (distanceABSq < minDistanceSq) {
        minDistanceSq = distanceABSq;
        closest = pointAB;
    }
    
    // 检查边bc
    Vector3 edgeBC = c - b;
    t = Math::clamp(Vector3::dot(point - b, edgeBC) / edgeBC.lengthSquared(), 0.0f, 1.0f);
    Vector3 pointBC = b + edgeBC * t;
    float distanceBCSq = (point - pointBC).lengthSquared();
    if (distanceBCSq < minDistanceSq) {
        minDistanceSq = distanceBCSq;
        closest = pointBC;
    }
    
    // 检查边ca
    Vector3 edgeCA = a - c;
    t = Math::clamp(Vector3::dot(point - c, edgeCA) / edgeCA.lengthSquared(), 0.0f, 1.0f);
    Vector3 pointCA = c + edgeCA * t;
    float distanceCASq = (point - pointCA).lengthSquared();
    if (distanceCASq < minDistanceSq) {
        closest = pointCA;
    }
    
    return closest;
}

float Triangle::distanceToPoint(const Vector3& point) const {
    Vector3 closest = closestPoint(point);
    return (point - closest).length();
}

// 碰撞检测方法
// 射线与三角面相交检测 - Möller-Trumbore算法
bool Triangle::intersects(const Ray& ray, float& t, Vector3& barycentric) const {
    const float EPSILON = Math::EPSILON;
    Vector3 edge1, edge2, h, s, q;
    float det, f, u, v;
    
    edge1 = this->b - this->a;
    edge2 = this->c - this->a;
    
    h = Vector3::cross(ray.direction, edge2);
    
    det = Vector3::dot(edge1, h);
    
    if (det > -EPSILON && det < EPSILON) {
        return false;
    }
    
    f = 1.0f / det;
    
    s = ray.origin - this->a;
    
    // 计算u = f * (s · h)
    u = f * Vector3::dot(s, h);
    
    // 如果u < 0或u > 1，交点不在三角面内
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    // 计算q = s × edge1
    q = Vector3::cross(s, edge1);
    
    // 计算v = f * (ray.direction · q)
    v = f * Vector3::dot(ray.direction, q);
    
    // 如果v < 0或u + v > 1，交点不在三角面内
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    // 计算t = f * (edge2 · q)
    t = f * Vector3::dot(edge2, q);
    
    // 只返回t > 0的交点（射线方向上的交点）
    if (t > EPSILON) {
        // 计算重心坐标
        barycentric = Vector3(1.0f - u - v, u, v);
        return true;
    }
    
    // 交点在射线起点后面
    return false;
}

bool Triangle::intersects(const Ray& ray, float& t) const {
    Vector3 barycentric;
    return intersects(ray, t, barycentric);
}

bool Triangle::intersects(const Sphere& sphere) const {
    // 检查球心到三角面的距离是否小于等于半径
    float distance = distanceToPoint(sphere.center);
    return distance <= sphere.radius;
}

bool Triangle::intersects(const Plane& plane) const {
    // 三角面与平面相交，如果三角面的三个顶点不在平面的同一侧
    float d1 = plane.distanceToPoint(a);
    float d2 = plane.distanceToPoint(b);
    float d3 = plane.distanceToPoint(c);
    
    // 检查是否有两个顶点在平面两侧
    return (d1 * d2 <= Math::EPSILON) || (d1 * d3 <= Math::EPSILON) || (d2 * d3 <= Math::EPSILON);
}

bool Triangle::intersects(const Bounds& bounds) const {
    // 检查三角面的包围盒是否与给定包围盒相交
    return getBounds().intersects(bounds);
}

bool Triangle::intersects(const Triangle& other) const {
    // 三角形与三角形相交检测（简单实现：检查两个三角面的包围盒是否相交）
    return getBounds().intersects(other.getBounds());
    // 注意：这里可以实现更精确的三角形相交检测算法，如SAT算法
}

// 静态方法
Triangle Triangle::fromPoints(const Vector3& a, const Vector3& b, const Vector3& c) {
    return Triangle(a, b, c);
}

float Triangle::barycentricToArea(const Vector3& barycentric, float triangleArea) {
    // 重心坐标的每个分量表示对应顶点对面的面积比例
    return barycentric.x * triangleArea;
}

} // namespace AstraAlgebra