// Plane.cpp - 平面类实现

#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Core/MathUtils.h>


namespace AstraAlgebra {

// 构造函数
Plane::Plane() : a(0.0f), b(0.0f), c(1.0f), d(0.0f) { // xy平面 (z=0)
}

Plane::Plane(float a, float b, float c, float d) : a(a), b(b), c(c), d(d) {
}

Plane::Plane(const Vector3& normal, float distance) {
    Vector3 norm = normal.normalized();
    a = norm.x;
    b = norm.y;
    c = norm.z;
    d = -distance;
}

Plane::Plane(const Vector3& normal, const Vector3& point) {
    Vector3 norm = normal.normalized();
    a = norm.x;
    b = norm.y;
    c = norm.z;
    d = -Vector3::dot(norm, point);
}

Plane::Plane(const Vector3& p0, const Vector3& p1, const Vector3& p2) {
    Vector3 v0 = p1 - p0;
    Vector3 v1 = p2 - p0;
    Vector3 normal = Vector3::cross(v0, v1).normalized();
    
    a = normal.x;
    b = normal.y;
    c = normal.z;
    d = -Vector3::dot(normal, p0);
}

Plane::Plane(const Plane& other) : a(other.a), b(other.b), c(other.c), d(other.d) {
}

Plane::Plane(Plane&& other) noexcept : a(other.a), b(other.b), c(other.c), d(other.d) {
    other.a = 0.0f;
    other.b = 0.0f;
    other.c = 1.0f;
    other.d = 0.0f;
}

// 赋值操作符
Plane& Plane::operator=(const Plane& other) {
    if (this != &other) {
        a = other.a;
        b = other.b;
        c = other.c;
        d = other.d;
    }
    return *this;
}

Plane& Plane::operator=(Plane&& other) noexcept {
    if (this != &other) {
        a = other.a;
        b = other.b;
        c = other.c;
        d = other.d;
        
        other.a = 0.0f;
        other.b = 0.0f;
        other.c = 1.0f;
        other.d = 0.0f;
    }
    return *this;
}

// 平面方法
Vector3 Plane::getNormal() const {
    return Vector3(a, b, c).normalized();
}

float Plane::getDistance() const {
    return -d / Vector3(a, b, c).length();
}

Plane Plane::normalized() const {
    Plane result = *this;
    result.normalize();
    return result;
}

Plane& Plane::normalize() {
    float length = Vector3(a, b, c).length();
    if (!Math::approximatelyEqual(length, 0.0f)) {
        float invLength = 1.0f / length;
        a *= invLength;
        b *= invLength;
        c *= invLength;
        d *= invLength;
    }
    return *this;
}

float Plane::distanceToPoint(const Vector3& point) const {
    return a * point.x + b * point.y + c * point.z + d;
}

Vector3 Plane::projectPoint(const Vector3& point) const {
    float distance = distanceToPoint(point);
    Vector3 normal = getNormal();
    return point - normal * distance;
}

bool Plane::isFrontFacing(const Vector3& direction) const {
    return Vector3::dot(getNormal(), direction) <= 0.0f;
}

// 碰撞检测方法
bool Plane::intersects(const Ray& ray, float& t) const {
    // 射线与平面相交检测
    // 平面方程：ax + by + cz + d = 0
    // 射线方程：r(t) = origin + t * direction
    
    float denom = a * ray.direction.x + b * ray.direction.y + c * ray.direction.z;
    
    // 如果射线与平面平行，不相交
    if (Math::approximatelyEqual(denom, 0.0f)) {
        return false;
    }
    
    // 计算交点参数t
    t = -(a * ray.origin.x + b * ray.origin.y + c * ray.origin.z + d) / denom;
    
    // 只返回t > 0的交点（射线方向上的交点）
    return t > Math::EPSILON;
}

bool Plane::intersects(const Vector3& point) const {
    return Math::approximatelyEqual(distanceToPoint(point), 0.0f);
}

bool Plane::intersects(const Plane& other, Ray& intersection) const {
    Vector3 n1(a, b, c);
    Vector3 n2(other.a, other.b, other.c);
    Vector3 n1xn2 = Vector3::cross(n1, n2);
    
    float denom = n1xn2.lengthSquared();
    if (Math::approximatelyEqual(denom, 0.0f)) {
        return false; // 平面平行
    }
    
    // 找到交线上的一个点
    Vector3 p;
    if (std::abs(n1xn2.z) > std::abs(n1xn2.x) && std::abs(n1xn2.z) > std::abs(n1xn2.y)) {
        // 选择z=0平面进行求解
        float invDenom = 1.0f / n1xn2.z;
        p.x = (b * other.d - other.b * d) * invDenom;
        p.y = (other.a * d - a * other.d) * invDenom;
        p.z = 0.0f;
    } else if (std::abs(n1xn2.y) > std::abs(n1xn2.x)) {
        // 选择y=0平面进行求解
        float invDenom = 1.0f / n1xn2.y;
        p.x = (other.c * d - c * other.d) * invDenom;
        p.y = 0.0f;
        p.z = (a * other.d - other.a * d) * invDenom;
    } else {
        // 选择x=0平面进行求解
        float invDenom = 1.0f / n1xn2.x;
        p.x = 0.0f;
        p.y = (c * other.d - other.c * d) * invDenom;
        p.z = (other.b * d - b * other.d) * invDenom;
    }
    
    intersection = Ray(p, n1xn2.normalized());
    return true;
}

bool Plane::intersects(const Sphere& sphere) const {
    float distance = std::abs(distanceToPoint(sphere.center));
    return distance <= sphere.radius;
}

bool Plane::intersects(const Bounds& bounds) const {
    // 使用分离轴定理检查AABB与平面是否相交
    Vector3 normal = getNormal();
    
    // 计算AABB在平面法向量方向上的投影区间
    float minProj = normal.x * bounds.min.x + normal.y * bounds.min.y + normal.z * bounds.min.z;
    float maxProj = normal.x * bounds.max.x + normal.y * bounds.max.y + normal.z * bounds.max.z;
    
    // 平面的位置在投影区间中的位置是 -d
    float planeProj = -d;
    
    // 检查投影区间是否与平面相交
    return !(planeProj < minProj - Math::EPSILON || planeProj > maxProj + Math::EPSILON);
}

// 静态方法
Plane Plane::fromPoints(const Vector3& p0, const Vector3& p1, const Vector3& p2) {
    return Plane(p0, p1, p2);
}

Plane Plane::xyPlane() {
    return Plane(0.0f, 0.0f, 1.0f, 0.0f);
}

Plane Plane::xzPlane() {
    return Plane(0.0f, 1.0f, 0.0f, 0.0f);
}

Plane Plane::yzPlane() {
    return Plane(1.0f, 0.0f, 0.0f, 0.0f);
}

} // namespace AstraAlgebra