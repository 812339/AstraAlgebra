// Line.cpp - 线段类实现

#include <AstraAlgebra/Geometry/Line.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <cmath>

namespace AstraAlgebra {

Line::Line(const Vector3& start, const Vector3& end) : start(start), end(end) {
}

float Line::length() const {
    return (end - start).length();
}

float Line::lengthSquared() const {
    return (end - start).lengthSquared();
}

Vector3 Line::direction() const {
    return (end - start).normalized();
}

Vector3 Line::midpoint() const {
    return (start + end) * 0.5f;
}

Vector3 Line::at(float t) const {
    return start + (end - start) * t;
}

float Line::distanceTo(const Vector3& point) const {
    return std::sqrt(distanceSquaredTo(point));
}

float Line::distanceSquaredTo(const Vector3& point) const {
    Vector3 lineVec = end - start;
    Vector3 pointVec = point - start;
    
    float lineLengthSquared = lineVec.lengthSquared();
    if (lineLengthSquared < Math::EPSILON) {
        // 线段退化为点
        return pointVec.lengthSquared();
    }
    
    float t = Math::clamp(pointVec.dot(lineVec) / lineLengthSquared, 0.0f, 1.0f);
    Vector3 closestPoint = start + lineVec * t;
    Vector3 distanceVec = point - closestPoint;
    
    return distanceVec.lengthSquared();
}

Vector3 Line::closestPointTo(const Vector3& point) const {
    Vector3 lineVec = end - start;
    Vector3 pointVec = point - start;
    
    float lineLengthSquared = lineVec.lengthSquared();
    if (lineLengthSquared < Math::EPSILON) {
        // 线段退化为点
        return start;
    }
    
    float t = Math::clamp(pointVec.dot(lineVec) / lineLengthSquared, 0.0f, 1.0f);
    return start + lineVec * t;
}

bool Line::intersects(const Ray& ray, float& tLine, float& tRay) const {
    Vector3 lineVec = end - start;
    Vector3 rayDir = ray.direction;
    
    Vector3 cross = lineVec.cross(rayDir);
    float crossLengthSquared = cross.lengthSquared();
    
    if (crossLengthSquared < Math::EPSILON) {
        // 线段和射线平行
        return false;
    }
    
    Vector3 startDiff = ray.origin - start;
    Vector3 startCross = startDiff.cross(rayDir);
    
    tLine = startCross.dot(cross) / crossLengthSquared;
    if (tLine < 0.0f || tLine > 1.0f) {
        return false;
    }
    
    Vector3 startCrossLine = startDiff.cross(lineVec);
    tRay = startCrossLine.dot(cross) / crossLengthSquared;
    
    return tRay >= 0.0f; // 射线只在正方向有意义
}

bool Line::intersects(const Plane& plane, float& t) const {
    Vector3 lineVec = end - start;
    Vector3 normal = plane.getNormal();
    float denom = normal.dot(lineVec);
    
    if (std::abs(denom) < Math::EPSILON) {
        // 线段和平面平行
        return false;
    }
    
    float numerator = plane.distanceToPoint(start);
    t = -numerator / denom;
    
    return t >= 0.0f && t <= 1.0f;
}

bool Line::intersects(const Sphere& sphere) const {
    float closestDistanceSquared = distanceSquaredTo(sphere.center);
    return closestDistanceSquared <= sphere.radius * sphere.radius;
}

bool Line::intersects(const Triangle& triangle, Vector3& intersectionPoint) const {
    // 使用Möller-Trumbore算法直接计算线段与三角形的相交
    Vector3 edge1 = triangle.b - triangle.a;
    Vector3 edge2 = triangle.c - triangle.a;
    Vector3 lineVec = end - start;
    Vector3 h = lineVec.cross(edge2);
    float a = edge1.dot(h);
    
    if (std::abs(a) < Math::EPSILON) {
        return false; // 线段与三角形平行
    }
    
    float f = 1.0f / a;
    Vector3 s = start - triangle.a;
    float u = f * s.dot(h);
    
    if (u < 0.0f || u > 1.0f) {
        return false;
    }
    
    Vector3 q = s.cross(edge1);
    float v = f * lineVec.dot(q);
    
    if (v < 0.0f || u + v > 1.0f) {
        return false;
    }
    
    float t = f * edge2.dot(q);
    
    if (t >= 0.0f && t <= 1.0f) {
        intersectionPoint = start + lineVec * t;
        return true;
    }
    
    return false;
}

bool Line::intersects(const Bounds& bounds) const {
    // 优化的线段与AABB相交检测（基于Slab方法）
    Vector3 min = bounds.min;
    Vector3 max = bounds.max;
    
    // 检查线段的两个端点是否在边界盒内
    if (bounds.contains(start) || bounds.contains(end)) {
        return true;
    }
    
    Vector3 origin = start;
    Vector3 dir = end - start;
    
    float tmin = 0.0f;
    float tmax = 1.0f;
    
    // X轴
    if (std::abs(dir.x) > Math::EPSILON) {
        float tx1 = (min.x - origin.x) / dir.x;
        float tx2 = (max.x - origin.x) / dir.x;
        tmin = std::max(tmin, std::min(tx1, tx2));
        tmax = std::min(tmax, std::max(tx1, tx2));
    } else {
        // 线段平行于X轴
        if (origin.x < min.x || origin.x > max.x) {
            return false;
        }
    }
    
    // Y轴
    if (std::abs(dir.y) > Math::EPSILON) {
        float ty1 = (min.y - origin.y) / dir.y;
        float ty2 = (max.y - origin.y) / dir.y;
        tmin = std::max(tmin, std::min(ty1, ty2));
        tmax = std::min(tmax, std::max(ty1, ty2));
    } else {
        // 线段平行于Y轴
        if (origin.y < min.y || origin.y > max.y) {
            return false;
        }
    }
    
    // Z轴
    if (std::abs(dir.z) > Math::EPSILON) {
        float tz1 = (min.z - origin.z) / dir.z;
        float tz2 = (max.z - origin.z) / dir.z;
        tmin = std::max(tmin, std::min(tz1, tz2));
        tmax = std::min(tmax, std::max(tz1, tz2));
    } else {
        // 线段平行于Z轴
        if (origin.z < min.z || origin.z > max.z) {
            return false;
        }
    }
    
    return tmax >= tmin;
}

bool Line::intersects(const Line& other, Vector3& intersectionPoint) const {
    Vector3 line1Vec = end - start;
    Vector3 line2Vec = other.end - other.start;
    
    Vector3 cross = line1Vec.cross(line2Vec);
    float crossLengthSquared = cross.lengthSquared();
    
    if (crossLengthSquared < Math::EPSILON) {
        // 线段平行
        return false;
    }
    
    Vector3 startDiff = other.start - start;
    float t1 = startDiff.cross(line2Vec).dot(cross) / crossLengthSquared;
    float t2 = startDiff.cross(line1Vec).dot(cross) / crossLengthSquared;
    
    if (t1 >= 0.0f && t1 <= 1.0f && t2 >= 0.0f && t2 <= 1.0f) {
        intersectionPoint = start + line1Vec * t1;
        return true;
    }
    
    return false;
}

Ray Line::toRay() const {
    return Ray(start, direction());
}

Line Line::fromPointDirection(const Vector3& point, const Vector3& direction, float length) {
    Vector3 normalizedDir = direction.normalized();
    return Line(point, point + normalizedDir * length);
}

Line Line::fromPoints(const Vector3* points, size_t count) {
    if (count == 0) {
        return Line();
    }
    
    Vector3 min = points[0];
    Vector3 max = points[0];
    
    for (size_t i = 1; i < count; ++i) {
        const Vector3& point = points[i];
        min = Vector3::min(min, point);
        max = Vector3::max(max, point);
    }
    
    return Line(min, max);
}

// 检查线段是否包含点
bool Line::contains(const Vector3& point) const {
    float distanceSq = distanceSquaredTo(point);
    return distanceSq < Math::EPSILON * Math::EPSILON;
}

// 反转线段的方向
Line Line::reversed() const {
    return Line(end, start);
}

// 延长线段
Line Line::extended(float startExtension, float endExtension) const {
    Vector3 dir = direction();
    return Line(start - dir * startExtension, end + dir * endExtension);
}

// 计算线段的包围盒
Bounds Line::bounds() const {
    Vector3 points[] = {start, end};
    return Bounds::fromPoints(points, 2);
}

} // namespace AstraAlgebra