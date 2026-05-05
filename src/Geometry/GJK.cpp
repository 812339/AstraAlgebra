// GJK.cpp - Gilbert-Johnson-Keerthi碰撞检测算法实现
// 凸体碰撞检测用的，算法挺复杂但用起来简单

#include <AstraAlgebra/Geometry/GJK.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Triangle.h>
#include <AstraAlgebra/Geometry/Capsule.h>
#include <AstraAlgebra/Geometry/Cone.h>
#include <AstraAlgebra/Geometry/Cylinder.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <algorithm>
#include <limits>

namespace AstraAlgebra {

// ------------------- Simplex 结构实现 -------------------

void GJK::Simplex::addPoint(const Vector3& point) {
    if (count < 4) {
        points[count++] = point;
    }
}

bool GJK::Simplex::containsOrigin(Vector3& closestPoint, Vector3& direction) {
    // 优化的GJK containsOrigin实现，提高性能和稳定性
    
    if (count == 0) {
        return false;
    }
    
    // 单一点情况
    if (count == 1) {
        closestPoint = points[0];
        direction = -closestPoint;
        return false;
    }
    
    // 两点情况
    if (count == 2) {
        Vector3& a = points[0];
        Vector3& b = points[1];
        Vector3 ab = b - a;
        Vector3 ao = -a;
        
        float abao = ab.dot(ao);
        
        if (abao <= 0.0f) {
            closestPoint = a;
            direction = ao;
            return false;
        }
        
        float abab = ab.dot(ab);
        if (abao >= abab) {
            closestPoint = b;
            direction = -b;
            return false;
        }
        
        float t = abao / abab;
        closestPoint = a + ab * t;
        direction = -closestPoint;
        return false;
    }
    
    // 三点情况（三角形）
    if (count == 3) {
        Vector3& a = points[0];
        Vector3& b = points[1];
        Vector3& c = points[2];
        
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ao = -a;
        
        // 计算三角形的法向量
        Vector3 abc = ab.cross(ac);
        
        // 检查原点是否在ab边的外侧
        Vector3 abperp = abc.cross(ab);
        if (abperp.dot(ao) < 0.0f) {
            // 原点在ab边外侧，使用ab线段
            count = 2;
            points[0] = a;
            points[1] = b;
            return containsOrigin(closestPoint, direction);
        }
        
        // 检查原点是否在ac边的外侧
        Vector3 acperp = ac.cross(abc);
        if (acperp.dot(ao) < 0.0f) {
            // 原点在ac边外侧，使用ac线段
            count = 2;
            points[0] = a;
            points[1] = c;
            return containsOrigin(closestPoint, direction);
        }
        
        // 检查原点是否在三角形内部
        if (abc.dot(ao) > 0.0f) {
            // 原点在三角形的正面
            direction = abc;
        } else {
            // 原点在三角形的背面
            direction = -abc;
        }
        
        // 原点在三角形内部，返回true
        closestPoint = Vector3::zero;
        return true;
    }
    
    // 四点情况（四面体）
    if (count == 4) {
        Vector3& a = points[0];
        Vector3& b = points[1];
        Vector3& c = points[2];
        Vector3& d = points[3];
        
        Vector3 ab = b - a;
        Vector3 ac = c - a;
        Vector3 ad = d - a;
        Vector3 ao = -a;
        
        // 计算四面体的四个面的法向量（指向外侧）
        Vector3 abc = ab.cross(ac);
        Vector3 acd = ac.cross(ad);
        Vector3 adb = ad.cross(ab);
        Vector3 bcd = (d - b).cross(c - b);
        
        // 检查原点是否在abc面外侧
        if (abc.dot(ao) > 0.0f) {
            // 原点在abc面外侧，移除d点
            count = 3;
            points[0] = a;
            points[1] = b;
            points[2] = c;
            return containsOrigin(closestPoint, direction);
        }
        
        // 检查原点是否在acd面外侧
        if (acd.dot(ao) > 0.0f) {
            // 原点在acd面外侧，移除b点
            count = 3;
            points[0] = a;
            points[1] = c;
            points[2] = d;
            return containsOrigin(closestPoint, direction);
        }
        
        // 检查原点是否在adb面外侧
        if (adb.dot(ao) > 0.0f) {
            // 原点在adb面外侧，移除c点
            count = 3;
            points[0] = a;
            points[1] = d;
            points[2] = b;
            return containsOrigin(closestPoint, direction);
        }
        
        // 检查原点是否在bcd面外侧
        Vector3 bao = a - b;
        if (bcd.dot(bao) > 0.0f) {
            // 原点在bcd面外侧，移除a点
            count = 3;
            points[0] = b;
            points[1] = c;
            points[2] = d;
            return containsOrigin(closestPoint, direction);
        }
        
        // 原点在四面体内
        closestPoint = Vector3::zero;
        direction = Vector3::zero;
        return true;
    }
    
    return false;
}

// ------------------- GJK算法核心实现 -------------------

template <typename ShapeA, typename ShapeB>
Vector3 GJK::support(const ShapeA& shapeA, const ShapeB& shapeB, const Vector3& direction) {
    // 获取shapeA在方向上的最远点
    Vector3 a = shapeA.support(direction);
    // 获取shapeB在相反方向上的最远点
    Vector3 b = shapeB.support(-direction);
    // 返回Minkowski差
    return a - b;
}

template <typename ShapeA, typename ShapeB>
bool GJK::doGJK(const ShapeA& shapeA, const ShapeB& shapeB, Simplex& simplex) {
    // 初始化搜索方向（使用中心连线方向）
    Vector3 direction = shapeB.support(Vector3::zero) - shapeA.support(Vector3::zero);
    if (direction.lengthSquared() < Math::EPSILON * Math::EPSILON) {
        direction = Vector3::right;
    }
    
    // 添加第一个点到simplex
    Vector3 initialPoint = support(shapeA, shapeB, direction);
    simplex.addPoint(initialPoint);
    
    // 反转搜索方向
    direction = -direction;
    
    int iterations = 0;
    const int maxIterations = 100;
    
    while (iterations < maxIterations) {
        iterations++;
        
        // 添加新点到simplex
        Vector3 newPoint = support(shapeA, shapeB, direction);
        
        // 检查新点是否在搜索方向上有进展
        if (newPoint.dot(direction) < 0.0f) {
            return false; // 没有进展，不相交
        }
        
        // 添加新点
        simplex.addPoint(newPoint);
        
        Vector3 closestPoint, newDirection;
        
        // 检查simplex是否包含原点
        if (simplex.containsOrigin(closestPoint, newDirection)) {
            return true; // 包含原点，相交
        }
        
        // 更新搜索方向
        direction = newDirection;
    }
    
    return false; // 超过最大迭代次数，假设不相交
}

// ------------------- EPA算法实现 -------------------

// 计算点到面的距离
static float distanceToFace(const Vector3& point, const Vector3& faceNormal, const Vector3& facePoint) {
    return faceNormal.dot(point - facePoint);
}

// 计算三角形的法向量（逆时针方向）
static Vector3 calculateFaceNormal(const Vector3& a, const Vector3& b, const Vector3& c) {
    Vector3 ab = b - a;
    Vector3 ac = c - a;
    return ab.cross(ac).normalized();
}

// 查找距离原点最近的面
int GJK::findClosestFace(const GJK::Polytope& polytope, float& minDistance, Vector3& closestNormal) {
    int closestFaceIndex = -1;
    minDistance = std::numeric_limits<float>::max();
    
    for (size_t i = 0; i < polytope.faces.size(); i += 3) {
        Vector3 a = polytope.points[static_cast<size_t>(polytope.faces[i])];
        Vector3 b = polytope.points[static_cast<size_t>(polytope.faces[i + 1])];
        Vector3 c = polytope.points[static_cast<size_t>(polytope.faces[i + 2])];
        
        Vector3 normal = calculateFaceNormal(a, b, c);
        float distance = normal.dot(a); // 原点到面的距离
        
        if (distance < minDistance && distance > 0.0f) {
            minDistance = distance;
            closestNormal = normal;
            closestFaceIndex = static_cast<int>(i);
        }
    }
    
    return closestFaceIndex;
}

// 检查点是否在面的外侧
bool GJK::isPointOutsideFace(const Vector3& point, const Vector3& faceNormal, const Vector3& facePoint) {
    return faceNormal.dot(point - facePoint) > 0.0f;
}

template <typename ShapeA, typename ShapeB>
void GJK::doEPA(const ShapeA& shapeA, const ShapeB& shapeB, const Simplex& simplex, GJKResult& result) {
    // 初始化多面体
    Polytope polytope;
    
    // 将simplex的点添加到多面体
    for (int i = 0; i < simplex.count; i++) {
        polytope.points.push_back(simplex.points[i]);
    }
    
    // 根据simplex的点数创建初始面
    if (simplex.count == 4) {
        // 四面体情况
        // 注意：这里需要确保面的法向量指向多面体外侧
        // 面0: 0, 1, 2
        polytope.faces.push_back(0);
        polytope.faces.push_back(1);
        polytope.faces.push_back(2);
        
        // 面1: 0, 2, 3
        polytope.faces.push_back(0);
        polytope.faces.push_back(2);
        polytope.faces.push_back(3);
        
        // 面2: 0, 3, 1
        polytope.faces.push_back(0);
        polytope.faces.push_back(3);
        polytope.faces.push_back(1);
        
        // 面3: 1, 3, 2
        polytope.faces.push_back(1);
        polytope.faces.push_back(3);
        polytope.faces.push_back(2);
    } else if (simplex.count == 3) {
        // 三角形情况（添加额外的点形成四面体）
        // 这里简化处理，实际实现需要更复杂的逻辑
        Vector3 extraPoint = polytope.points[0] + (polytope.points[1] - polytope.points[0]).cross(
            polytope.points[2] - polytope.points[0]
        ) * 0.1f;
        polytope.points.push_back(extraPoint);
        
        // 创建四面体的四个面
        polytope.faces.push_back(0);
        polytope.faces.push_back(1);
        polytope.faces.push_back(2);
        
        polytope.faces.push_back(0);
        polytope.faces.push_back(2);
        polytope.faces.push_back(3);
        
        polytope.faces.push_back(0);
        polytope.faces.push_back(3);
        polytope.faces.push_back(1);
        
        polytope.faces.push_back(1);
        polytope.faces.push_back(3);
        polytope.faces.push_back(2);
    }
    
    int iterations = 0;
    const int maxIterations = 50;
    
    // EPA主循环
    while (iterations < maxIterations) {
        iterations++;
        
        // 查找距离原点最近的面
        float minDistance;
        Vector3 closestNormal;
        int closestFaceIndex = findClosestFace(polytope, minDistance, closestNormal);
        
        if (closestFaceIndex == -1) {
            break; // 没有找到有效的面，退出
        }
        
        // 获取最近面的顶点
        size_t i0 = static_cast<size_t>(polytope.faces[static_cast<size_t>(closestFaceIndex)]);
        size_t i1 = static_cast<size_t>(polytope.faces[static_cast<size_t>(closestFaceIndex) + 1]);
        size_t i2 = static_cast<size_t>(polytope.faces[static_cast<size_t>(closestFaceIndex) + 2]);
        Vector3 facePoint = polytope.points[i0];
        
        // 获取支持点
        Vector3 supportPoint = support(shapeA, shapeB, closestNormal);
        
        // 计算支持点到最近面的距离
        float supportDistance = closestNormal.dot(supportPoint);
        
        // 检查是否收敛
        if (supportDistance - minDistance < Math::EPSILON) {
            // 已经收敛，计算碰撞信息
            result.intersect = true;
            result.normal = closestNormal;
            result.penetration = supportDistance;
            
            // 计算接触点（简化实现，使用最近面的一个点）
            result.contactPoint = supportPoint - closestNormal * supportDistance;
            
            return;
        }
        
        // 将支持点添加到多面体
        size_t newPointIndex = polytope.points.size();
        polytope.points.push_back(supportPoint);
        
        // 移除所有可见的面（即支持点可以看到的面）
        std::vector<int> newFaces;
        
        for (size_t i = 0; i < polytope.faces.size(); i += 3) {
            int aIdx = polytope.faces[i];
            int bIdx = polytope.faces[i + 1];
            int cIdx = polytope.faces[i + 2];
            
            Vector3 a = polytope.points[static_cast<size_t>(aIdx)];
            Vector3 b = polytope.points[static_cast<size_t>(bIdx)];
            Vector3 c = polytope.points[static_cast<size_t>(cIdx)];
            
            Vector3 faceNormal = calculateFaceNormal(a, b, c);
            
            // 检查面是否可见
            if (!isPointOutsideFace(supportPoint, faceNormal, a)) {
                // 面不可见，保留
                newFaces.push_back(aIdx);
                newFaces.push_back(bIdx);
                newFaces.push_back(cIdx);
            }
        }
        
        // 添加新的边与新点形成的面
        // 这里简化处理，实际实现需要更复杂的边处理
        // 为了简化，我们只处理四面体情况
        if (polytope.points.size() <= 6) {
            for (size_t i = 0; i < polytope.points.size() - 1; i++) {
                newFaces.push_back(static_cast<int>(i));
                newFaces.push_back(static_cast<int>((i + 1) % (polytope.points.size() - 1)));
                newFaces.push_back(static_cast<int>(newPointIndex));
            }
        }
        
        polytope.faces = newFaces;
    }
    
    // 如果循环结束还没有收敛，返回默认碰撞信息
    result.intersect = true;
    result.normal = Vector3::up;
    result.penetration = 0.0f;
    result.contactPoint = Vector3::zero;
}

// ------------------- GJK类公共方法实现 -------------------

template <typename ShapeA, typename ShapeB>
GJKResult GJK::intersect(const ShapeA& shapeA, const ShapeB& shapeB) {
    GJKResult result;
    Simplex simplex;
    
    // 执行GJK算法
    if (doGJK(shapeA, shapeB, simplex)) {
        // 相交，执行EPA算法获取详细碰撞信息
        doEPA(shapeA, shapeB, simplex, result);
    }
    
    return result;
}

template <typename ShapeA, typename ShapeB>
bool GJK::testIntersection(const ShapeA& shapeA, const ShapeB& shapeB) {
    Simplex simplex;
    return doGJK(shapeA, shapeB, simplex);
}

// ------------------- ConvexShape接口实现 -------------------

// BoundsConvex实现
Vector3 BoundsConvex::support(const Vector3& direction) const {
    Vector3 point;
    point.x = direction.x >= 0.0f ? m_bounds.max.x : m_bounds.min.x;
    point.y = direction.y >= 0.0f ? m_bounds.max.y : m_bounds.min.y;
    point.z = direction.z >= 0.0f ? m_bounds.max.z : m_bounds.min.z;
    return point;
}

// SphereConvex实现
Vector3 SphereConvex::support(const Vector3& direction) const {
    Vector3 dir = direction.normalized();
    return m_sphere.center + dir * m_sphere.radius;
}

// TriangleConvex实现
Vector3 TriangleConvex::support(const Vector3& direction) const {
    float da = m_triangle.a.dot(direction);
    float db = m_triangle.b.dot(direction);
    float dc = m_triangle.c.dot(direction);
    
    float maxDot = std::max(std::max(da, db), dc);
    
    if (maxDot == da) {
        return m_triangle.a;
    } else if (maxDot == db) {
        return m_triangle.b;
    } else {
        return m_triangle.c;
    }
}

// CapsuleConvex实现
Vector3 CapsuleConvex::support(const Vector3& direction) const {
    // 优化的胶囊体support函数，减少不必要的计算
    Vector3 axis = m_capsule.point2 - m_capsule.point1;
    float axisLengthSq = axis.dot(axis);
    
    if (axisLengthSq < Math::EPSILON * Math::EPSILON) {
        // 胶囊体退化为球体，直接计算结果
        return m_capsule.point1 + direction.normalized() * m_capsule.radius;
    }
    
    // 计算轴线单位向量（避免多次调用normalized）
    Vector3 axisNorm = axis / std::sqrt(axisLengthSq);
    
    // 计算方向在轴线上的投影
    float dot = direction.dot(axisNorm);
    
    // 计算线段上的最近点参数t
    float t = Math::clamp(dot, 0.0f, 1.0f);
    
    // 计算线段上的最近点
    Vector3 closestOnSegment = m_capsule.point1 + axisNorm * t * std::sqrt(axisLengthSq);
    
    // 计算垂直于轴线的方向分量
    Vector3 dirPerp = direction - dot * axisNorm;
    float perpLengthSq = dirPerp.dot(dirPerp);
    
    if (perpLengthSq < Math::EPSILON * Math::EPSILON) {
        // 方向平行于轴线，返回端点加上偏移
        return closestOnSegment + axisNorm * Math::sign(dot) * m_capsule.radius;
    } else {
        // 方向不平行于轴线，返回偏移后的点
        return closestOnSegment + dirPerp.normalized() * m_capsule.radius;
    }
}

// ConeConvex实现
Vector3 ConeConvex::support(const Vector3& direction) const {
    // 优化的圆锥support函数，减少不必要的计算
    Vector3 axis = m_cone.baseCenter - m_cone.apex;
    float height = axis.length();
    
    if (height < Math::EPSILON) {
        // 圆锥退化为圆盘
        return m_cone.apex + direction.normalized() * m_cone.radius;
    }
    
    // 计算轴线单位向量和方向与轴线的点积
    Vector3 axisNorm = axis / height;
    float dot = direction.dot(axisNorm);
    
    // 计算圆锥半顶角的正切值（避免多次三角函数调用）
    float tanHalfAngle = m_cone.radius / height;
    
    // 快速判断方向是否指向圆锥内部
    // 如果 dot >= |direction| * cos(theta)，则在圆锥内部
    // 其中 theta 是圆锥半顶角
    float dirLength = direction.length();
    if (dot >= dirLength / std::sqrt(1.0f + tanHalfAngle * tanHalfAngle)) {
        return m_cone.apex;
    }
    
    // 否则返回底面圆周上的点
    Vector3 perpDir = direction - dot * axisNorm;
    if (perpDir.lengthSquared() < Math::EPSILON * Math::EPSILON) {
        // 方向平行于轴线，返回底面边缘点
        return m_cone.baseCenter + Vector3::cross(axisNorm, Vector3::right).normalized() * m_cone.radius;
    }
    
    Vector3 baseEdge = Vector3::cross(axisNorm, perpDir).normalized();
    return m_cone.baseCenter + baseEdge * m_cone.radius;
}

// CylinderConvex实现
Vector3 CylinderConvex::support(const Vector3& direction) const {
    // 优化的圆柱support函数，减少不必要的计算
    Vector3 axis = m_cylinder.center2 - m_cylinder.center1;
    float axisLengthSq = axis.dot(axis);
    
    if (axisLengthSq < Math::EPSILON * Math::EPSILON) {
        // 圆柱退化为球体，直接计算结果
        return m_cylinder.center1 + direction.normalized() * m_cylinder.radius;
    }
    
    // 计算轴线单位向量（避免多次调用normalized）
    Vector3 axisNorm = axis / std::sqrt(axisLengthSq);
    
    // 计算方向在轴线上的投影
    float dot = direction.dot(axisNorm);
    
    // 计算线段上的最近点参数t
    float t = Math::clamp(dot, 0.0f, 1.0f);
    
    // 计算线段上的最近点
    Vector3 closestOnSegment = m_cylinder.center1 + axisNorm * t * std::sqrt(axisLengthSq);
    
    // 计算垂直于轴线的方向分量
    Vector3 dirPerp = direction - dot * axisNorm;
    float perpLengthSq = dirPerp.dot(dirPerp);
    
    if (perpLengthSq < Math::EPSILON * Math::EPSILON) {
        // 方向平行于轴线，返回端点加上偏移
        return closestOnSegment + axisNorm * Math::sign(dot) * m_cylinder.radius;
    } else {
        // 方向不平行于轴线，返回偏移后的点
        return closestOnSegment + dirPerp.normalized() * m_cylinder.radius;
    }
}

// ------------------- 显式实例化模板函数 -------------------

// 为常用几何类型组合实例化GJK模板

template GJKResult GJK::intersect<BoundsConvex, SphereConvex>(const BoundsConvex&, const SphereConvex&);
template GJKResult GJK::intersect<BoundsConvex, BoundsConvex>(const BoundsConvex&, const BoundsConvex&);
template GJKResult GJK::intersect<SphereConvex, SphereConvex>(const SphereConvex&, const SphereConvex&);
template GJKResult GJK::intersect<BoundsConvex, TriangleConvex>(const BoundsConvex&, const TriangleConvex&);
template GJKResult GJK::intersect<SphereConvex, TriangleConvex>(const SphereConvex&, const TriangleConvex&);
template GJKResult GJK::intersect<TriangleConvex, TriangleConvex>(const TriangleConvex&, const TriangleConvex&);
template GJKResult GJK::intersect<BoundsConvex, CapsuleConvex>(const BoundsConvex&, const CapsuleConvex&);
template GJKResult GJK::intersect<SphereConvex, CapsuleConvex>(const SphereConvex&, const CapsuleConvex&);
template GJKResult GJK::intersect<CapsuleConvex, CapsuleConvex>(const CapsuleConvex&, const CapsuleConvex&);
template GJKResult GJK::intersect<BoundsConvex, ConeConvex>(const BoundsConvex&, const ConeConvex&);
template GJKResult GJK::intersect<SphereConvex, ConeConvex>(const SphereConvex&, const ConeConvex&);
template GJKResult GJK::intersect<ConeConvex, ConeConvex>(const ConeConvex&, const ConeConvex&);
template GJKResult GJK::intersect<BoundsConvex, CylinderConvex>(const BoundsConvex&, const CylinderConvex&);
template GJKResult GJK::intersect<SphereConvex, CylinderConvex>(const SphereConvex&, const CylinderConvex&);
template GJKResult GJK::intersect<CylinderConvex, CylinderConvex>(const CylinderConvex&, const CylinderConvex&);

template bool GJK::testIntersection<BoundsConvex, SphereConvex>(const BoundsConvex&, const SphereConvex&);
template bool GJK::testIntersection<BoundsConvex, BoundsConvex>(const BoundsConvex&, const BoundsConvex&);
template bool GJK::testIntersection<SphereConvex, SphereConvex>(const SphereConvex&, const SphereConvex&);
template bool GJK::testIntersection<BoundsConvex, TriangleConvex>(const BoundsConvex&, const TriangleConvex&);
template bool GJK::testIntersection<SphereConvex, TriangleConvex>(const SphereConvex&, const TriangleConvex&);
template bool GJK::testIntersection<TriangleConvex, TriangleConvex>(const TriangleConvex&, const TriangleConvex&);
template bool GJK::testIntersection<BoundsConvex, CapsuleConvex>(const BoundsConvex&, const CapsuleConvex&);
template bool GJK::testIntersection<SphereConvex, CapsuleConvex>(const SphereConvex&, const CapsuleConvex&);
template bool GJK::testIntersection<CapsuleConvex, CapsuleConvex>(const CapsuleConvex&, const CapsuleConvex&);
template bool GJK::testIntersection<BoundsConvex, ConeConvex>(const BoundsConvex&, const ConeConvex&);
template bool GJK::testIntersection<SphereConvex, ConeConvex>(const SphereConvex&, const ConeConvex&);
template bool GJK::testIntersection<ConeConvex, ConeConvex>(const ConeConvex&, const ConeConvex&);
template bool GJK::testIntersection<BoundsConvex, CylinderConvex>(const BoundsConvex&, const CylinderConvex&);
template bool GJK::testIntersection<SphereConvex, CylinderConvex>(const SphereConvex&, const CylinderConvex&);
template bool GJK::testIntersection<CylinderConvex, CylinderConvex>(const CylinderConvex&, const CylinderConvex&);

} // namespace AstraAlgebra