// GJK - Gilbert-Johnson-Keerthi碰撞检测算法，凸体碰撞检测用的
// 这算法挺复杂的，但用起来很简单，调用一下就行

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <vector>

namespace AstraAlgebra {

// 前向声明
class Bounds;
class Sphere;
class Triangle;
class Capsule;
class Cone;
class Cylinder;

// 凸体接口，GJK算法要用
class ConvexShape {
public:
    virtual ~ConvexShape() = default;
    
    // 支持函数：给个方向，返回形状上最远的点
    virtual Vector3 support(const Vector3& direction) const = 0;
};

// GJK碰撞检测结果
struct GJKResult {
    bool intersect;      // 碰上了没
    Vector3 normal;      // 碰撞法线
    float penetration;   // 穿透深度
    Vector3 contactPoint;// 接触点
    
    GJKResult() : intersect(false), normal(Vector3::zero), penetration(0.0f), contactPoint(Vector3::zero) {}
};

// GJK算法实现
class GJK {
private:
    // 四面体，GJK算法用的
    struct Simplex {
        Vector3 points[4];  // 四个点
        int count;          // 当前点数（1-4）
        
        Simplex() : count(0) {}
        
        void addPoint(const Vector3& point);  // 加点
        bool containsOrigin(Vector3& closestPoint, Vector3& direction);  // 包不包含原点
    };
    
    // EPA算法的多面体
    struct Polytope {
        std::vector<Vector3> points;
        std::vector<int> faces;
        std::vector<float> depths;
    };
    
    // Minkowski差的支持点
    template <typename ShapeA, typename ShapeB>
    static Vector3 support(const ShapeA& shapeA, const ShapeB& shapeB, const Vector3& direction);
    
    // GJK算法主体
    template <typename ShapeA, typename ShapeB>
    static bool doGJK(const ShapeA& shapeA, const ShapeB& shapeB, Simplex& simplex);
    
    // 找离原点最近的面
    static int findClosestFace(const Polytope& polytope, float& minDistance, Vector3& closestNormal);
    
    // 检查点是不是在面外侧
    static bool isPointOutsideFace(const Vector3& point, const Vector3& faceNormal, const Vector3& facePoint);
    
    // EPA算法，算精确碰撞信息
    template <typename ShapeA, typename ShapeB>
    static void doEPA(const ShapeA& shapeA, const ShapeB& shapeB, const Simplex& simplex, GJKResult& result);
    
public:
    // 检测碰撞
    template <typename ShapeA, typename ShapeB>
    static GJKResult intersect(const ShapeA& shapeA, const ShapeB& shapeB);
    
    // 只检测碰没碰，不算详细信息
    template <typename ShapeA, typename ShapeB>
    static bool testIntersection(const ShapeA& shapeA, const ShapeB& shapeB);
};

// 几何类的ConvexShape包装
class BoundsConvex : public ConvexShape {
private:
    const Bounds& m_bounds;
public:
    BoundsConvex(const Bounds& bounds) : m_bounds(bounds) {}
    Vector3 support(const Vector3& direction) const override;
};

class SphereConvex : public ConvexShape {
private:
    const Sphere& m_sphere;
public:
    SphereConvex(const Sphere& sphere) : m_sphere(sphere) {}
    Vector3 support(const Vector3& direction) const override;
};

class TriangleConvex : public ConvexShape {
private:
    const Triangle& m_triangle;
public:
    TriangleConvex(const Triangle& triangle) : m_triangle(triangle) {}
    Vector3 support(const Vector3& direction) const override;
};

class CapsuleConvex : public ConvexShape {
private:
    const Capsule& m_capsule;
public:
    CapsuleConvex(const Capsule& capsule) : m_capsule(capsule) {}
    Vector3 support(const Vector3& direction) const override;
};

class ConeConvex : public ConvexShape {
private:
    const Cone& m_cone;
public:
    ConeConvex(const Cone& cone) : m_cone(cone) {}
    Vector3 support(const Vector3& direction) const override;
};

class CylinderConvex : public ConvexShape {
private:
    const Cylinder& m_cylinder;
public:
    CylinderConvex(const Cylinder& cylinder) : m_cylinder(cylinder) {}
    Vector3 support(const Vector3& direction) const override;
};

// 全局函数，用起来方便点
template <typename ShapeA, typename ShapeB>
bool gjkIntersect(const ShapeA& shapeA, const ShapeB& shapeB) {
    return GJK::testIntersection(shapeA, shapeB);
}

template <typename ShapeA, typename ShapeB>
GJKResult gjkIntersectionDetails(const ShapeA& shapeA, const ShapeB& shapeB) {
    return GJK::intersect(shapeA, shapeB);
}

} // namespace AstraAlgebra
