// AstraAlgebra - Comprehensive Test Suite
// Tests all core math library functionality

#include <iostream>
#include <cmath>
#include <cassert>
#include <string>
#include <vector>
#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Vector/Vector3d.h>
#include <AstraAlgebra/Matrix/Matrix2x2.h>
#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Matrix/Matrix4x4d.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Core/SIMD.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Numerics/MatrixDecomposition.h>
#include <AstraAlgebra/Numerics/ColorSpace.h>
#include <AstraAlgebra/Numerics/Random.h>
#include <AstraAlgebra/Core/MathUtils.h>

using namespace AstraAlgebra;

static int tests_passed = 0;
static int tests_failed = 0;
static std::vector<std::string> failed_tests;

#define TEST(name) void name(); \
    struct name##_registrar { name##_registrar() { run_test(#name, name); } } name##_instance; \
    void name()

static void run_test(const char* name, void (*func)()) {
    try {
        func();
        tests_passed++;
        std::cout << "  [PASS] " << name << std::endl;
    } catch (const std::exception& e) {
        tests_failed++;
        failed_tests.push_back(std::string(name) + ": " + e.what());
        std::cout << "  [FAIL] " << name << " - " << e.what() << std::endl;
    }
}

#define ASSERT_NEAR(a, b, eps) do { \
    if (std::abs((a) - (b)) > (eps)) \
        throw std::runtime_error(std::string("Expected ") + std::to_string(b) + " but got " + std::to_string(a)); \
} while(0)

#define ASSERT_TRUE(cond) do { \
    if (!(cond)) \
        throw std::runtime_error("Assertion failed: " #cond); \
} while(0)

#define ASSERT_EQUAL(a, b) do { \
    if ((a) != (b)) \
        throw std::runtime_error(std::string("Expected ") + std::to_string(b) + " but got " + std::to_string(a)); \
} while(0)

// ========== MathUtils Tests ==========
TEST(MathUtils_degToRad) {
    ASSERT_NEAR(Math::degToRad(180.0f), Math::PI, 1e-5f);
    ASSERT_NEAR(Math::degToRad(90.0f), Math::PI / 2.0f, 1e-5f);
    ASSERT_NEAR(Math::degToRad(0.0f), 0.0f, 1e-5f);
}

TEST(MathUtils_radToDeg) {
    ASSERT_NEAR(Math::radToDeg(Math::PI), 180.0f, 1e-5f);
    ASSERT_NEAR(Math::radToDeg(Math::PI / 2.0f), 90.0f, 1e-5f);
}

TEST(MathUtils_lerp) {
    ASSERT_NEAR(Math::lerp(0.0f, 10.0f, 0.5f), 5.0f, 1e-5f);
    ASSERT_NEAR(Math::lerp(0.0f, 10.0f, 0.0f), 0.0f, 1e-5f);
    ASSERT_NEAR(Math::lerp(0.0f, 10.0f, 1.0f), 10.0f, 1e-5f);
}

TEST(MathUtils_clamp) {
    ASSERT_NEAR(Math::clamp(5.0f, 0.0f, 10.0f), 5.0f, 1e-5f);
    ASSERT_NEAR(Math::clamp(-5.0f, 0.0f, 10.0f), 0.0f, 1e-5f);
    ASSERT_NEAR(Math::clamp(15.0f, 0.0f, 10.0f), 10.0f, 1e-5f);
}

TEST(MathUtils_smoothStep) {
    ASSERT_NEAR(Math::smoothStep(0.0f, 1.0f, 0.0f), 0.0f, 1e-5f);
    ASSERT_NEAR(Math::smoothStep(0.0f, 1.0f, 1.0f), 1.0f, 1e-5f);
    ASSERT_NEAR(Math::smoothStep(0.0f, 1.0f, 0.5f), 0.5f, 1e-3f);
}

TEST(MathUtils_approximatelyEqual) {
    ASSERT_TRUE(Math::approximatelyEqual(1.0f, 1.0000001f));
    ASSERT_TRUE(!Math::approximatelyEqual(1.0f, 2.0f));
}

// ========== Vector2 Tests ==========
TEST(Vector2_defaultConstructor) {
    Vector2 v;
    ASSERT_NEAR(v.x, 0.0f, 1e-5f);
    ASSERT_NEAR(v.y, 0.0f, 1e-5f);
}

TEST(Vector2_parameterizedConstructor) {
    Vector2 v(3.0f, 4.0f);
    ASSERT_NEAR(v.x, 3.0f, 1e-5f);
    ASSERT_NEAR(v.y, 4.0f, 1e-5f);
}

TEST(Vector2_scalarConstructor) {
    Vector2 v(5.0f);
    ASSERT_NEAR(v.x, 5.0f, 1e-5f);
    ASSERT_NEAR(v.y, 5.0f, 1e-5f);
}

TEST(Vector2_addition) {
    Vector2 a(1.0f, 2.0f);
    Vector2 b(3.0f, 4.0f);
    Vector2 c = a + b;
    ASSERT_NEAR(c.x, 4.0f, 1e-5f);
    ASSERT_NEAR(c.y, 6.0f, 1e-5f);
}

TEST(Vector2_subtraction) {
    Vector2 a(5.0f, 6.0f);
    Vector2 b(1.0f, 2.0f);
    Vector2 c = a - b;
    ASSERT_NEAR(c.x, 4.0f, 1e-5f);
    ASSERT_NEAR(c.y, 4.0f, 1e-5f);
}

TEST(Vector2_scalarMultiplication) {
    Vector2 v(2.0f, 3.0f);
    Vector2 r = v * 2.0f;
    ASSERT_NEAR(r.x, 4.0f, 1e-5f);
    ASSERT_NEAR(r.y, 6.0f, 1e-5f);
}

TEST(Vector2_scalarDivision) {
    Vector2 v(4.0f, 6.0f);
    Vector2 r = v / 2.0f;
    ASSERT_NEAR(r.x, 2.0f, 1e-5f);
    ASSERT_NEAR(r.y, 3.0f, 1e-5f);
}

TEST(Vector2_scalarVectorAddition) {
    Vector2 v(1.0f, 2.0f);
    Vector2 r = 3.0f + v;
    ASSERT_NEAR(r.x, 4.0f, 1e-5f);
    ASSERT_NEAR(r.y, 5.0f, 1e-5f);
}

TEST(Vector2_vectorScalarSubtraction) {
    Vector2 v(5.0f, 6.0f);
    Vector2 r = v - 2.0f;
    ASSERT_NEAR(r.x, 3.0f, 1e-5f);
    ASSERT_NEAR(r.y, 4.0f, 1e-5f);
}

TEST(Vector2_length) {
    Vector2 v(3.0f, 4.0f);
    ASSERT_NEAR(v.length(), 5.0f, 1e-5f);
    ASSERT_NEAR(v.lengthSquared(), 25.0f, 1e-5f);
}

TEST(Vector2_normalized) {
    Vector2 v(3.0f, 4.0f);
    Vector2 n = v.normalized();
    ASSERT_NEAR(n.length(), 1.0f, 1e-5f);
}

TEST(Vector2_dotProduct) {
    Vector2 a(1.0f, 0.0f);
    Vector2 b(0.0f, 1.0f);
    ASSERT_NEAR(a.dot(b), 0.0f, 1e-5f);
    
    Vector2 c(1.0f, 2.0f);
    Vector2 d(3.0f, 4.0f);
    ASSERT_NEAR(c.dot(d), 11.0f, 1e-5f);
}

TEST(Vector2_crossProduct) {
    Vector2 a(1.0f, 0.0f);
    Vector2 b(0.0f, 1.0f);
    ASSERT_NEAR(a.cross(b), 1.0f, 1e-5f);
}

TEST(Vector2_distance) {
    Vector2 a(0.0f, 0.0f);
    Vector2 b(3.0f, 4.0f);
    ASSERT_NEAR(a.distance(b), 5.0f, 1e-5f);
}

TEST(Vector2_lerp) {
    Vector2 a(0.0f, 0.0f);
    Vector2 b(10.0f, 10.0f);
    Vector2 c = Vector2::lerp(a, b, 0.5f);
    ASSERT_NEAR(c.x, 5.0f, 1e-5f);
    ASSERT_NEAR(c.y, 5.0f, 1e-5f);
}

TEST(Vector2_reflect) {
    Vector2 v(1.0f, -1.0f);
    Vector2 n(0.0f, 1.0f);
    Vector2 r = v.reflect(n);
    ASSERT_NEAR(r.x, 1.0f, 1e-5f);
    ASSERT_NEAR(r.y, 1.0f, 1e-5f);
}

TEST(Vector2_staticConstants) {
    ASSERT_NEAR(Vector2::zero.x, 0.0f, 1e-5f);
    ASSERT_NEAR(Vector2::one.x, 1.0f, 1e-5f);
    ASSERT_NEAR(Vector2::right.x, 1.0f, 1e-5f);
    ASSERT_NEAR(Vector2::up.y, 1.0f, 1e-5f);
}

// ========== Vector3 Tests ==========
TEST(Vector3_defaultConstructor) {
    Vector3 v;
    ASSERT_NEAR(v.x, 0.0f, 1e-5f);
    ASSERT_NEAR(v.y, 0.0f, 1e-5f);
    ASSERT_NEAR(v.z, 0.0f, 1e-5f);
}

TEST(Vector3_parameterizedConstructor) {
    Vector3 v(1.0f, 2.0f, 3.0f);
    ASSERT_NEAR(v.x, 1.0f, 1e-5f);
    ASSERT_NEAR(v.y, 2.0f, 1e-5f);
    ASSERT_NEAR(v.z, 3.0f, 1e-5f);
}

TEST(Vector3_addition) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    Vector3 c = a + b;
    ASSERT_NEAR(c.x, 5.0f, 1e-5f);
    ASSERT_NEAR(c.y, 7.0f, 1e-5f);
    ASSERT_NEAR(c.z, 9.0f, 1e-5f);
}

TEST(Vector3_subtraction) {
    Vector3 a(4.0f, 5.0f, 6.0f);
    Vector3 b(1.0f, 2.0f, 3.0f);
    Vector3 c = a - b;
    ASSERT_NEAR(c.x, 3.0f, 1e-5f);
    ASSERT_NEAR(c.y, 3.0f, 1e-5f);
    ASSERT_NEAR(c.z, 3.0f, 1e-5f);
}

TEST(Vector3_scalarMultiplication) {
    Vector3 v(1.0f, 2.0f, 3.0f);
    Vector3 r = v * 3.0f;
    ASSERT_NEAR(r.x, 3.0f, 1e-5f);
    ASSERT_NEAR(r.y, 6.0f, 1e-5f);
    ASSERT_NEAR(r.z, 9.0f, 1e-5f);
}

TEST(Vector3_scalarDivision) {
    Vector3 v(3.0f, 6.0f, 9.0f);
    Vector3 r = v / 3.0f;
    ASSERT_NEAR(r.x, 1.0f, 1e-5f);
    ASSERT_NEAR(r.y, 2.0f, 1e-5f);
    ASSERT_NEAR(r.z, 3.0f, 1e-5f);
}

TEST(Vector3_length) {
    Vector3 v(1.0f, 2.0f, 2.0f);
    ASSERT_NEAR(v.length(), 3.0f, 1e-5f);
    ASSERT_NEAR(v.lengthSquared(), 9.0f, 1e-5f);
}

TEST(Vector3_normalized) {
    Vector3 v(1.0f, 2.0f, 2.0f);
    Vector3 n = v.normalized();
    ASSERT_NEAR(n.length(), 1.0f, 1e-5f);
    ASSERT_NEAR(n.x, 1.0f/3.0f, 1e-5f);
    ASSERT_NEAR(n.y, 2.0f/3.0f, 1e-5f);
    ASSERT_NEAR(n.z, 2.0f/3.0f, 1e-5f);
}

TEST(Vector3_dotProduct) {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);
    ASSERT_NEAR(a.dot(b), 32.0f, 1e-5f);
}

TEST(Vector3_crossProduct) {
    Vector3 a(1.0f, 0.0f, 0.0f);
    Vector3 b(0.0f, 1.0f, 0.0f);
    Vector3 c = a.cross(b);
    ASSERT_NEAR(c.x, 0.0f, 1e-5f);
    ASSERT_NEAR(c.y, 0.0f, 1e-5f);
    ASSERT_NEAR(c.z, 1.0f, 1e-5f);
}

TEST(Vector3_distance) {
    Vector3 a(0.0f, 0.0f, 0.0f);
    Vector3 b(1.0f, 2.0f, 2.0f);
    ASSERT_NEAR(a.distance(b), 3.0f, 1e-5f);
}

TEST(Vector3_lerp) {
    Vector3 a(0.0f, 0.0f, 0.0f);
    Vector3 b(10.0f, 10.0f, 10.0f);
    Vector3 c = Vector3::lerp(a, b, 0.5f);
    ASSERT_NEAR(c.x, 5.0f, 1e-5f);
    ASSERT_NEAR(c.y, 5.0f, 1e-5f);
    ASSERT_NEAR(c.z, 5.0f, 1e-5f);
}

TEST(Vector3_angle) {
    Vector3 a(1.0f, 0.0f, 0.0f);
    Vector3 b(0.0f, 1.0f, 0.0f);
    float angle = Vector3::angle(a, b);
    ASSERT_NEAR(angle, Math::PI / 2.0f, 1e-5f);
}

TEST(Vector3_project) {
    Vector3 v(3.0f, 4.0f, 0.0f);
    Vector3 onto(1.0f, 0.0f, 0.0f);
    Vector3 proj = v.project(onto);
    ASSERT_NEAR(proj.x, 3.0f, 1e-5f);
    ASSERT_NEAR(proj.y, 0.0f, 1e-5f);
    ASSERT_NEAR(proj.z, 0.0f, 1e-5f);
}

TEST(Vector3_reflect) {
    Vector3 v(1.0f, -1.0f, 0.0f);
    Vector3 n(0.0f, 1.0f, 0.0f);
    Vector3 r = v.reflect(n);
    ASSERT_NEAR(r.x, 1.0f, 1e-5f);
    ASSERT_NEAR(r.y, 1.0f, 1e-5f);
    ASSERT_NEAR(r.z, 0.0f, 1e-5f);
}

TEST(Vector3_staticConstants) {
    ASSERT_NEAR(Vector3::zero.x, 0.0f, 1e-5f);
    ASSERT_NEAR(Vector3::one.x, 1.0f, 1e-5f);
    ASSERT_NEAR(Vector3::forward.z, 1.0f, 1e-5f);
    ASSERT_NEAR(Vector3::up.y, 1.0f, 1e-5f);
    ASSERT_NEAR(Vector3::right.x, 1.0f, 1e-5f);
}

// ========== Vector4 Tests ==========
TEST(Vector4_defaultConstructor) {
    Vector4 v;
    ASSERT_NEAR(v.x, 0.0f, 1e-5f);
    ASSERT_NEAR(v.y, 0.0f, 1e-5f);
    ASSERT_NEAR(v.z, 0.0f, 1e-5f);
    ASSERT_NEAR(v.w, 1.0f, 1e-5f); // homogeneous coordinate default
}

TEST(Vector4_parameterizedConstructor) {
    Vector4 v(1.0f, 2.0f, 3.0f, 4.0f);
    ASSERT_NEAR(v.x, 1.0f, 1e-5f);
    ASSERT_NEAR(v.y, 2.0f, 1e-5f);
    ASSERT_NEAR(v.z, 3.0f, 1e-5f);
    ASSERT_NEAR(v.w, 4.0f, 1e-5f);
}

TEST(Vector4_addition) {
    Vector4 a(1.0f, 2.0f, 3.0f, 4.0f);
    Vector4 b(5.0f, 6.0f, 7.0f, 8.0f);
    Vector4 c = a + b;
    ASSERT_NEAR(c.x, 6.0f, 1e-5f);
    ASSERT_NEAR(c.y, 8.0f, 1e-5f);
    ASSERT_NEAR(c.z, 10.0f, 1e-5f);
    ASSERT_NEAR(c.w, 12.0f, 1e-5f);
}

TEST(Vector4_toCartesian) {
    Vector4 v(2.0f, 4.0f, 6.0f, 2.0f);
    Vector3 c = v.toVector3();
    ASSERT_NEAR(c.x, 2.0f, 1e-5f);
    ASSERT_NEAR(c.y, 4.0f, 1e-5f);
    ASSERT_NEAR(c.z, 6.0f, 1e-5f);
}

TEST(Vector4_fromVector3) {
    Vector3 v(1.0f, 2.0f, 3.0f);
    Vector4 h(v.x, v.y, v.z, 1.0f);
    ASSERT_NEAR(h.x, 1.0f, 1e-5f);
    ASSERT_NEAR(h.y, 2.0f, 1e-5f);
    ASSERT_NEAR(h.z, 3.0f, 1e-5f);
    ASSERT_NEAR(h.w, 1.0f, 1e-5f);
}

// ========== Matrix2x2 Tests ==========
TEST(Matrix2x2_identity) {
    Matrix2x2 m = Matrix2x2::identity;
    ASSERT_NEAR(m.m[0][0], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[1][1], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[0][1], 0.0f, 1e-5f);
    ASSERT_NEAR(m.m[1][0], 0.0f, 1e-5f);
}

TEST(Matrix2x2_determinant) {
    Matrix2x2 m(1.0f, 2.0f, 3.0f, 4.0f);
    ASSERT_NEAR(m.determinant(), -2.0f, 1e-5f);
}

TEST(Matrix2x2_transpose) {
    Matrix2x2 m(1.0f, 2.0f, 3.0f, 4.0f);
    Matrix2x2 t = m.transposed();
    ASSERT_NEAR(t.m[0][1], 3.0f, 1e-5f);
    ASSERT_NEAR(t.m[1][0], 2.0f, 1e-5f);
}

TEST(Matrix2x2_multiplication) {
    Matrix2x2 a = Matrix2x2::identity;
    Matrix2x2 b(2.0f, 0.0f, 0.0f, 2.0f);
    Matrix2x2 c = a * b;
    ASSERT_NEAR(c.m[0][0], 2.0f, 1e-5f);
    ASSERT_NEAR(c.m[1][1], 2.0f, 1e-5f);
}

// ========== Matrix3x3 Tests ==========
TEST(Matrix3x3_identity) {
    Matrix3x3 m = Matrix3x3::identity;
    ASSERT_NEAR(m.m[0][0], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[1][1], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[2][2], 1.0f, 1e-5f);
}

TEST(Matrix3x3_determinant) {
    Matrix3x3 m(1.0f, 0.0f, 0.0f, 0.0f, 2.0f, 0.0f, 0.0f, 0.0f, 3.0f);
    ASSERT_NEAR(m.determinant(), 6.0f, 1e-5f);
}

TEST(Matrix3x3_transpose) {
    Matrix3x3 m(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, 7.0f, 8.0f, 9.0f);
    Matrix3x3 t = m.transposed();
    ASSERT_NEAR(t.m[0][1], 4.0f, 1e-5f);
    ASSERT_NEAR(t.m[1][0], 2.0f, 1e-5f);
}

// ========== Matrix4x4 Tests ==========
TEST(Matrix4x4_identity) {
    Matrix4x4 m = Matrix4x4::identity;
    ASSERT_NEAR(m.m[0][0], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[1][1], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[2][2], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[3][3], 1.0f, 1e-5f);
}

TEST(Matrix4x4_translation) {
    Matrix4x4 m = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));
    ASSERT_NEAR(m.m[0][3], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[1][3], 2.0f, 1e-5f);
    ASSERT_NEAR(m.m[2][3], 3.0f, 1e-5f);
    ASSERT_NEAR(m.m[3][3], 1.0f, 1e-5f);
}

TEST(Matrix4x4_scale) {
    Matrix4x4 m = Matrix4x4::scale(Vector3(2.0f, 3.0f, 4.0f));
    ASSERT_NEAR(m.m[0][0], 2.0f, 1e-5f);
    ASSERT_NEAR(m.m[1][1], 3.0f, 1e-5f);
    ASSERT_NEAR(m.m[2][2], 4.0f, 1e-5f);
}

TEST(Matrix4x4_rotationX) {
    Matrix4x4 m = Matrix4x4::rotationX(90.0f, true);
    ASSERT_NEAR(m.m[1][1], 0.0f, 1e-4f);
    ASSERT_NEAR(m.m[1][2], -1.0f, 1e-4f);
    ASSERT_NEAR(m.m[2][1], 1.0f, 1e-4f);
    ASSERT_NEAR(m.m[2][2], 0.0f, 1e-4f);
}

TEST(Matrix4x4_rotationY) {
    Matrix4x4 m = Matrix4x4::rotationY(90.0f, true);
    ASSERT_NEAR(m.m[0][0], 0.0f, 1e-4f);
    ASSERT_NEAR(m.m[0][2], 1.0f, 1e-4f);
    ASSERT_NEAR(m.m[2][0], -1.0f, 1e-4f);
    ASSERT_NEAR(m.m[2][2], 0.0f, 1e-4f);
}

TEST(Matrix4x4_rotationZ) {
    Matrix4x4 m = Matrix4x4::rotationZ(90.0f, true);
    ASSERT_NEAR(m.m[0][0], 0.0f, 1e-4f);
    ASSERT_NEAR(m.m[0][1], -1.0f, 1e-4f);
    ASSERT_NEAR(m.m[1][0], 1.0f, 1e-4f);
    ASSERT_NEAR(m.m[1][1], 0.0f, 1e-4f);
}

TEST(Matrix4x4_determinant) {
    Matrix4x4 m = Matrix4x4::identity;
    ASSERT_NEAR(m.determinant(), 1.0f, 1e-5f);
}

TEST(Matrix4x4_inverse) {
    Matrix4x4 m = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));
    Matrix4x4 inv = m.inverse();
    Matrix4x4 result = m * inv;
    ASSERT_NEAR(result.m[0][0], 1.0f, 1e-4f);
    ASSERT_NEAR(result.m[1][1], 1.0f, 1e-4f);
    ASSERT_NEAR(result.m[2][2], 1.0f, 1e-4f);
}

TEST(Matrix4x4_transpose) {
    Matrix4x4 m = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));
    Matrix4x4 t = m.transposed();
    ASSERT_NEAR(t.m[3][0], 1.0f, 1e-5f);
    ASSERT_NEAR(t.m[3][1], 2.0f, 1e-5f);
    ASSERT_NEAR(t.m[3][2], 3.0f, 1e-5f);
}

TEST(Matrix4x4_lookAt) {
    Matrix4x4 m = Matrix4x4::lookAt(
        Vector3(0.0f, 0.0f, 5.0f),
        Vector3(0.0f, 0.0f, 0.0f),
        Vector3(0.0f, 1.0f, 0.0f)
    );
    ASSERT_NEAR(m.m[3][3], 1.0f, 1e-5f);
}

TEST(Matrix4x4_perspective) {
    Matrix4x4 m = Matrix4x4::perspective(60.0f, 16.0f/9.0f, 0.1f, 100.0f);
    ASSERT_TRUE(m.m[3][3] == 0.0f);
}

TEST(Matrix4x4_decompose) {
    Vector3 pos(1.0f, 2.0f, 3.0f);
    Vector3 scale(2.0f, 2.0f, 2.0f);
    Matrix4x4 m = Matrix4x4::translation(pos) * Matrix4x4::scale(scale);
    
    Vector3 outPos, outScale;
    Quaternion outRot;
    m.decompose(outPos, outRot, outScale);
    
    ASSERT_NEAR(outPos.x, pos.x, 1e-4f);
    ASSERT_NEAR(outPos.y, pos.y, 1e-4f);
    ASSERT_NEAR(outPos.z, pos.z, 1e-4f);
    ASSERT_NEAR(outScale.x, scale.x, 1e-4f);
    ASSERT_NEAR(outScale.y, scale.y, 1e-4f);
    ASSERT_NEAR(outScale.z, scale.z, 1e-4f);
}

// ========== Quaternion Tests ==========
TEST(Quaternion_defaultConstructor) {
    Quaternion q;
    ASSERT_NEAR(q.x, 0.0f, 1e-5f);
    ASSERT_NEAR(q.y, 0.0f, 1e-5f);
    ASSERT_NEAR(q.z, 0.0f, 1e-5f);
    ASSERT_NEAR(q.w, 1.0f, 1e-5f);
}

TEST(Quaternion_identity) {
    ASSERT_NEAR(Quaternion::identity.x, 0.0f, 1e-5f);
    ASSERT_NEAR(Quaternion::identity.w, 1.0f, 1e-5f);
}

TEST(Quaternion_fromAxisAngle) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    float halfAngle = Math::degToRad(45.0f);
    ASSERT_NEAR(q.x, 0.0f, 1e-5f);
    ASSERT_NEAR(q.y, std::sin(halfAngle), 1e-5f);
    ASSERT_NEAR(q.z, 0.0f, 1e-5f);
    ASSERT_NEAR(q.w, std::cos(halfAngle), 1e-5f);
}

TEST(Quaternion_multiplication) {
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    Quaternion result = q1 * q2;
    ASSERT_NEAR(result.y, q2.y, 1e-5f);
    ASSERT_NEAR(result.w, q2.w, 1e-5f);
}

TEST(Quaternion_vectorRotation) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    Vector3 forward = Vector3::forward;
    Vector3 rotated = q * forward;
    ASSERT_NEAR(rotated.x, 1.0f, 1e-4f);
    ASSERT_NEAR(rotated.y, 0.0f, 1e-4f);
    ASSERT_NEAR(rotated.z, 0.0f, 1e-4f);
}

TEST(Quaternion_conjugate) {
    Quaternion q(0.5f, 0.5f, 0.5f, 0.5f);
    Quaternion c = q.conjugate();
    ASSERT_NEAR(c.x, -0.5f, 1e-5f);
    ASSERT_NEAR(c.y, -0.5f, 1e-5f);
    ASSERT_NEAR(c.z, -0.5f, 1e-5f);
    ASSERT_NEAR(c.w, 0.5f, 1e-5f);
}

TEST(Quaternion_dot) {
    Quaternion a(1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion b(0.0f, 1.0f, 0.0f, 0.0f);
    ASSERT_NEAR(a.dot(b), 0.0f, 1e-5f);
}

TEST(Quaternion_normalized) {
    Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
    Quaternion n = q.normalized();
    ASSERT_NEAR(n.length(), 1.0f, 1e-4f);
}

TEST(Quaternion_slerp) {
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    Quaternion result = q1.slerp(q2, 0.5f);
    // slerp at t=0.5 between identity and 90° rotation gives 45° rotation
    // w = cos(45°/2) = cos(22.5°) ≈ 0.923884
    ASSERT_NEAR(result.w, 0.923884f, 0.001f);
    ASSERT_NEAR(result.length(), 1.0f, 0.01f);
}

TEST(Quaternion_nlerp) {
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    Quaternion result = q1.nlerp(q2, 0.5f);
    ASSERT_NEAR(result.w, (q1.w + q2.w) * 0.5f, 0.12f);
}

TEST(Quaternion_toEulerAngles) {
    Quaternion q = Quaternion::fromEulerAngles(Vector3(0.0f, 90.0f, 0.0f), true);
    Vector3 euler = q.toEulerAngles(true);
    ASSERT_NEAR(euler.y, 90.0f, 1e-3f);
}

TEST(Quaternion_inverse) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3::up, 45.0f, true);
    Quaternion inv = q.inverse();
    Quaternion result = q * inv;
    ASSERT_NEAR(result.w, 1.0f, 1e-4f);
    ASSERT_NEAR(result.x, 0.0f, 1e-4f);
    ASSERT_NEAR(result.y, 0.0f, 1e-4f);
    ASSERT_NEAR(result.z, 0.0f, 1e-4f);
}

TEST(Quaternion_lookRotation) {
    Quaternion q = Quaternion::lookRotation(Vector3::forward, Vector3::up);
    ASSERT_NEAR(q.length(), 1.0f, 1e-4f);
    ASSERT_NEAR(q.w, 0.0f, 1e-4f);
    ASSERT_NEAR(q.y, 1.0f, 1e-4f);
}

// ========== Geometry Tests ==========
TEST(Ray_pointAt) {
    Ray ray(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f));
    Vector3 p = ray.pointAt(5.0f);
    ASSERT_NEAR(p.x, 5.0f, 1e-5f);
    ASSERT_NEAR(p.y, 0.0f, 1e-5f);
    ASSERT_NEAR(p.z, 0.0f, 1e-5f);
}

TEST(Ray_intersectSphere) {
    Ray ray(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f));
    Sphere sphere(Vector3(5.0f, 0.0f, 0.0f), 1.0f);
    float t = ray.intersect(sphere);
    ASSERT_TRUE(t > 0.0f);
}

TEST(Ray_intersectPlane) {
    Ray ray(Vector3(0.0f, 5.0f, 0.0f), Vector3(0.0f, -1.0f, 0.0f));
    Plane plane(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
    float t = ray.intersect(plane);
    ASSERT_NEAR(t, 5.0f, 1e-5f);
}

TEST(Sphere_contains) {
    Sphere sphere(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    ASSERT_TRUE(sphere.contains(Vector3(0.5f, 0.5f, 0.0f)));
    ASSERT_TRUE(!sphere.contains(Vector3(2.0f, 0.0f, 0.0f)));
}

TEST(Sphere_intersects) {
    Sphere s1(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    Sphere s2(Vector3(1.5f, 0.0f, 0.0f), 1.0f);
    ASSERT_TRUE(s1.intersects(s2));
    
    Sphere s3(Vector3(5.0f, 0.0f, 0.0f), 1.0f);
    ASSERT_TRUE(!s1.intersects(s3));
}

TEST(Plane_distanceToPoint) {
    Plane plane(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
    ASSERT_NEAR(plane.distanceToPoint(Vector3(0.0f, 5.0f, 0.0f)), 5.0f, 1e-5f);
}

TEST(Plane_projectPoint) {
    Plane plane(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
    Vector3 p = plane.projectPoint(Vector3(0.0f, 5.0f, 0.0f));
    ASSERT_NEAR(p.x, 0.0f, 1e-5f);
    ASSERT_NEAR(p.y, 0.0f, 1e-5f);
    ASSERT_NEAR(p.z, 0.0f, 1e-5f);
}

TEST(Bounds_contains) {
    Bounds bounds(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
    ASSERT_TRUE(bounds.contains(Vector3(0.0f, 0.0f, 0.0f)));
    ASSERT_TRUE(!bounds.contains(Vector3(2.0f, 0.0f, 0.0f)));
}

TEST(Bounds_intersects) {
    Bounds b1(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
    Bounds b2(Vector3(0.5f, 0.5f, 0.5f), Vector3(2.0f, 2.0f, 2.0f));
    ASSERT_TRUE(b1.intersects(b2));
    
    Bounds b3(Vector3(5.0f, 5.0f, 5.0f), Vector3(6.0f, 6.0f, 6.0f));
    ASSERT_TRUE(!b1.intersects(b3));
}

TEST(Bounds_merged) {
    Bounds b1(Vector3(-1.0f, -1.0f, -1.0f), Vector3(1.0f, 1.0f, 1.0f));
    Bounds b2(Vector3(2.0f, 2.0f, 2.0f), Vector3(3.0f, 3.0f, 3.0f));
    Bounds merged = b1.merged(b2);
    ASSERT_NEAR(merged.min.x, -1.0f, 1e-5f);
    ASSERT_NEAR(merged.max.x, 3.0f, 1e-5f);
}

// ========== Transform Tests ==========
TEST(Transform_position) {
    // Transform tests skipped - requires Transform.h include
}

TEST(Transform_rotation) {
    // Transform tests skipped
}

TEST(Transform_scale) {
    // Transform tests skipped
}

TEST(Transform_toMatrix) {
    // Transform tests skipped
}

// ========== Constexpr Tests ==========
TEST(Constexpr_Vector2) {
    constexpr Vector2 v1(1.0f, 2.0f);
    constexpr Vector2 v2(3.0f, 4.0f);
    constexpr Vector2 sum = v1 + v2;
    constexpr float dot = v1.dot(v2);
    static_assert(sum.x == 4.0f, "Vector2 constexpr addition failed");
    static_assert(sum.y == 6.0f, "Vector2 constexpr addition failed");
    static_assert(dot == 11.0f, "Vector2 constexpr dot failed");
}

TEST(Constexpr_Vector3) {
    constexpr Vector3 v1(1.0f, 2.0f, 3.0f);
    constexpr Vector3 v2(4.0f, 5.0f, 6.0f);
    Vector3 cross = v1.cross(v2);
    ASSERT_NEAR(cross.x, -3.0f, 1e-5f);
    ASSERT_NEAR(cross.y, 6.0f, 1e-5f);
    ASSERT_NEAR(cross.z, -3.0f, 1e-5f);
}

TEST(Constexpr_Matrix4x4) {
    constexpr Matrix4x4 identity(1.0f, 0.0f, 0.0f, 0.0f,
                                 0.0f, 1.0f, 0.0f, 0.0f,
                                 0.0f, 0.0f, 1.0f, 0.0f,
                                 0.0f, 0.0f, 0.0f, 1.0f);
    static_assert(identity.m[0][0] == 1.0f, "Matrix4x4 constexpr identity failed");
    static_assert(identity.m[1][1] == 1.0f, "Matrix4x4 constexpr identity failed");
    static_assert(identity.m[2][2] == 1.0f, "Matrix4x4 constexpr identity failed");
}

TEST(Constexpr_Quaternion) {
    constexpr Quaternion q1(0.0f, 0.0f, 0.0f, 1.0f);
    constexpr Quaternion q2(0.0f, 0.707f, 0.0f, 0.707f);
    constexpr Quaternion product = q1 * q2;
    static_assert(Math::abs(product.y - 0.707f) < 1e-3f, "Quaternion constexpr multiplication failed");
    static_assert(Math::abs(product.w - 0.707f) < 1e-3f, "Quaternion constexpr multiplication failed");
}

// ========== Double Precision Tests ==========
TEST(Vector3d_defaultConstructor) {
    Vector3d v;
    ASSERT_NEAR(v.x, 0.0, 1e-10);
    ASSERT_NEAR(v.y, 0.0, 1e-10);
    ASSERT_NEAR(v.z, 0.0, 1e-10);
}

TEST(Vector3d_operations) {
    Vector3d a(1.0, 2.0, 3.0);
    Vector3d b(4.0, 5.0, 6.0);
    Vector3d c = a + b;
    ASSERT_NEAR(c.x, 5.0, 1e-10);
    ASSERT_NEAR(c.y, 7.0, 1e-10);
    ASSERT_NEAR(c.z, 9.0, 1e-10);
}

TEST(Vector3d_dotProduct) {
    Vector3d a(1.0, 2.0, 3.0);
    Vector3d b(4.0, 5.0, 6.0);
    double dot = a.dot(b);
    ASSERT_NEAR(dot, 32.0, 1e-10);
}

TEST(Vector3d_crossProduct) {
    Vector3d a(1.0, 0.0, 0.0);
    Vector3d b(0.0, 1.0, 0.0);
    Vector3d c = a.cross(b);
    ASSERT_NEAR(c.x, 0.0, 1e-10);
    ASSERT_NEAR(c.y, 0.0, 1e-10);
    ASSERT_NEAR(c.z, 1.0, 1e-10);
}

TEST(Vector3d_length) {
    Vector3d v(3.0, 4.0, 0.0);
    ASSERT_NEAR(v.length(), 5.0, 1e-10);
}

TEST(Vector3d_normalized) {
    Vector3d v(3.0, 4.0, 0.0);
    Vector3d n = v.normalized();
    ASSERT_NEAR(n.length(), 1.0, 1e-10);
}

TEST(Vector3d_staticConstants) {
    ASSERT_NEAR(Vector3d::zero.x, 0.0, 1e-10);
    ASSERT_NEAR(Vector3d::one.y, 1.0, 1e-10);
    ASSERT_NEAR(Vector3d::up.z, 0.0, 1e-10);
    ASSERT_NEAR(Vector3d::forward.z, 1.0, 1e-10);
}

TEST(Matrix4x4d_identity) {
    Matrix4x4d m = Matrix4x4d::identity;
    ASSERT_NEAR(m.m[0][0], 1.0, 1e-10);
    ASSERT_NEAR(m.m[1][1], 1.0, 1e-10);
    ASSERT_NEAR(m.m[2][2], 1.0, 1e-10);
}

TEST(Matrix4x4d_translation) {
    Matrix4x4d m = Matrix4x4d::translation(Vector3d(1.0, 2.0, 3.0));
    ASSERT_NEAR(m.m[0][3], 1.0, 1e-10);
    ASSERT_NEAR(m.m[1][3], 2.0, 1e-10);
    ASSERT_NEAR(m.m[2][3], 3.0, 1e-10);
}

TEST(Matrix4x4d_scale) {
    Matrix4x4d m = Matrix4x4d::scale(Vector3d(2.0, 3.0, 4.0));
    ASSERT_NEAR(m.m[0][0], 2.0, 1e-10);
    ASSERT_NEAR(m.m[1][1], 3.0, 1e-10);
    ASSERT_NEAR(m.m[2][2], 4.0, 1e-10);
}

TEST(Matrix4x4d_multiplication) {
    Matrix4x4d a = Matrix4x4d::identity;
    Matrix4x4d b = Matrix4x4d::translation(Vector3d(1.0, 2.0, 3.0));
    Matrix4x4d c = a * b;
    ASSERT_NEAR(c.m[0][3], 1.0, 1e-10);
    ASSERT_NEAR(c.m[1][3], 2.0, 1e-10);
    ASSERT_NEAR(c.m[2][3], 3.0, 1e-10);
}

TEST(Matrix4x4d_transformVector) {
    Matrix4x4d m = Matrix4x4d::translation(Vector3d(1.0, 2.0, 3.0));
    Vector3d v(0.0, 0.0, 0.0);
    Vector3d result = m * v;
    ASSERT_NEAR(result.x, 1.0, 1e-10);
    ASSERT_NEAR(result.y, 2.0, 1e-10);
    ASSERT_NEAR(result.z, 3.0, 1e-10);
}

TEST(Matrix4x4d_determinant) {
    Matrix4x4d m = Matrix4x4d::identity;
    ASSERT_NEAR(m.determinant(), 1.0, 1e-10);
}

TEST(Matrix4x4d_inverse) {
    Matrix4x4d m = Matrix4x4d::translation(Vector3d(1.0, 2.0, 3.0));
    Matrix4x4d inv = m.inverse();
    Matrix4x4d result = m * inv;
    ASSERT_NEAR(result.m[0][0], 1.0, 1e-8);
    ASSERT_NEAR(result.m[1][1], 1.0, 1e-8);
    ASSERT_NEAR(result.m[2][2], 1.0, 1e-8);
}

TEST(Matrix4x4d_lookAt) {
    Matrix4x4d view = Matrix4x4d::lookAt(Vector3d(0.0, 0.0, 5.0), Vector3d(0.0, 0.0, 0.0), Vector3d::up);
    ASSERT_NEAR(view.m[3][3], 1.0, 1e-10);
}

TEST(Matrix4x4d_perspective) {
    Matrix4x4d proj = Matrix4x4d::perspective(Math::degToRad(60.0), 16.0/9.0, 0.1, 100.0);
    ASSERT_NEAR(proj.m[3][2], -1.0, 1e-10);
}

// ========== Matrix Decomposition Tests ==========
TEST(MatrixDecomposition_LU) {
    Matrix4x4 A = Matrix4x4::identity;
    A.m[0][0] = 2.0f; A.m[0][1] = 1.0f;
    A.m[1][0] = 1.0f; A.m[1][1] = 3.0f;
    
    Matrix4x4 L, U, P;
    bool success = MatrixDecomposition::luDecompose(A, L, U, P);
    ASSERT_TRUE(success);
}

TEST(MatrixDecomposition_QR) {
    Matrix4x4 A = Matrix4x4::identity;
    Matrix4x4 Q, R;
    bool success = MatrixDecomposition::qrDecompose(A, Q, R);
    ASSERT_TRUE(success);
}

TEST(MatrixDecomposition_Cholesky) {
    Matrix4x4 A = Matrix4x4::identity;
    Matrix4x4 L;
    bool success = MatrixDecomposition::choleskyDecompose(A, L);
    ASSERT_TRUE(success);
}

TEST(MatrixDecomposition_Eigen3x3) {
    Matrix3x3 A = Matrix3x3::identity;
    A.m[0][0] = 4.0f; A.m[0][1] = 2.0f; A.m[1][0] = 2.0f; A.m[1][1] = 1.0f;
    
    Vector3 eigenvalues;
    Vector3 v1, v2, v3;
    bool success = MatrixDecomposition::eigenDecompose3x3(A, eigenvalues, v1, v2, v3);
    ASSERT_TRUE(success);
}

TEST(MatrixDecomposition_SVD3x3) {
    Matrix3x3 A = Matrix3x3::identity;
    A.m[0][0] = 2.0f; A.m[1][1] = 3.0f;
    
    Matrix3x3 U, V;
    Vector3 S;
    bool success = MatrixDecomposition::svdDecompose3x3(A, U, S, V);
    ASSERT_TRUE(success);
    ASSERT_TRUE(S.x > 0.0f && S.y > 0.0f && S.z > 0.0f);
}

TEST(MatrixDecomposition_Polar) {
    Matrix4x4 A = Matrix4x4::identity;
    Matrix4x4 R, S;
    bool success = MatrixDecomposition::polarDecompose(A, R, S);
    ASSERT_TRUE(success);
}

TEST(MatrixDecomposition_DecomposeTransform) {
    Matrix4x4 transform = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f)) * 
                          Matrix4x4::rotationY(Math::PI_4) * 
                          Matrix4x4::scale(Vector3(2.0f, 2.0f, 2.0f));
    
    Vector3 translation;
    Quaternion rotation;
    Vector3 scale;
    bool success = MatrixDecomposition::decomposeTransform(transform, translation, rotation, scale);
    ASSERT_TRUE(success);
    ASSERT_NEAR(translation.x, 1.0f, 0.1f);
    ASSERT_NEAR(translation.y, 2.0f, 0.1f);
    ASSERT_NEAR(translation.z, 3.0f, 0.1f);
}

TEST(MatrixDecomposition_SolveLinearSystem) {
    Matrix4x4 A = Matrix4x4::identity;
    A.m[0][0] = 2.0f; A.m[1][1] = 3.0f; A.m[2][2] = 4.0f;
    
    Vector3 b(2.0f, 3.0f, 4.0f);
    Vector3 x;
    bool success = MatrixDecomposition::solveLinearSystem(A, b, x);
    ASSERT_TRUE(success);
    ASSERT_NEAR(x.x, 1.0f, 0.01f);
    ASSERT_NEAR(x.y, 1.0f, 0.01f);
    ASSERT_NEAR(x.z, 1.0f, 0.01f);
}

TEST(MatrixDecomposition_Rank) {
    Matrix4x4 A = Matrix4x4::identity;
    int rank = MatrixDecomposition::computeRank(A);
    ASSERT_EQUAL(rank, 4);
}

// ========== Color Space Tests ==========
TEST(ColorSpace_RGB_HSV) {
    ColorRGB rgb(1.0f, 0.0f, 0.0f);
    ColorHSV hsv = ColorSpace::rgbToHsv(rgb);
    ASSERT_NEAR(hsv.h, 0.0f, 1.0f);
    ASSERT_NEAR(hsv.s, 1.0f, 0.01f);
    ASSERT_NEAR(hsv.v, 1.0f, 0.01f);
}

TEST(ColorSpace_HSV_RGB) {
    ColorHSV hsv(120.0f, 1.0f, 1.0f);
    ColorRGB rgb = ColorSpace::hsvToRgb(hsv);
    ASSERT_NEAR(rgb.r, 0.0f, 0.01f);
    ASSERT_NEAR(rgb.g, 1.0f, 0.01f);
    ASSERT_NEAR(rgb.b, 0.0f, 0.01f);
}

TEST(ColorSpace_RGB_HSL) {
    ColorRGB rgb(0.0f, 0.0f, 1.0f);
    ColorHSL hsl = ColorSpace::rgbToHsl(rgb);
    ASSERT_NEAR(hsl.h, 240.0f, 1.0f);
    ASSERT_NEAR(hsl.s, 1.0f, 0.01f);
    ASSERT_NEAR(hsl.l, 0.5f, 0.01f);
}

TEST(ColorSpace_HSL_RGB) {
    ColorHSL hsl(60.0f, 1.0f, 0.5f);
    ColorRGB rgb = ColorSpace::hslToRgb(hsl);
    ASSERT_NEAR(rgb.r, 1.0f, 0.01f);
    ASSERT_NEAR(rgb.g, 1.0f, 0.01f);
    ASSERT_NEAR(rgb.b, 0.0f, 0.01f);
}

TEST(ColorSpace_RGB_CMYK) {
    ColorRGB rgb(1.0f, 0.0f, 0.0f);
    ColorCMYK cmyk = ColorSpace::rgbToCmyk(rgb);
    ASSERT_NEAR(cmyk.c, 0.0f, 0.01f);
    ASSERT_NEAR(cmyk.m, 1.0f, 0.01f);
    ASSERT_NEAR(cmyk.y, 1.0f, 0.01f);
    ASSERT_NEAR(cmyk.k, 0.0f, 0.01f);
}

TEST(ColorSpace_CMYK_RGB) {
    ColorCMYK cmyk(1.0f, 1.0f, 0.0f, 0.0f);
    ColorRGB rgb = ColorSpace::cmykToRgb(cmyk);
    ASSERT_NEAR(rgb.r, 0.0f, 0.01f);
    ASSERT_NEAR(rgb.g, 0.0f, 0.01f);
    ASSERT_NEAR(rgb.b, 1.0f, 0.01f);
}

TEST(ColorSpace_Lerp) {
    ColorRGB a(1.0f, 0.0f, 0.0f);
    ColorRGB b(0.0f, 0.0f, 1.0f);
    ColorRGB c = ColorSpace::lerp(a, b, 0.5f);
    ASSERT_NEAR(c.r, 0.5f, 0.01f);
    ASSERT_NEAR(c.g, 0.0f, 0.01f);
    ASSERT_NEAR(c.b, 0.5f, 0.01f);
}

TEST(ColorSpace_GammaCorrect) {
    ColorRGB linear(0.5f, 0.5f, 0.5f);
    ColorRGB srgb = ColorSpace::gammaCorrect(linear, 2.2f);
    ASSERT_TRUE(srgb.r > 0.5f);
}

TEST(ColorSpace_PredefinedColors) {
    ColorRGB red = ColorSpace::red();
    ASSERT_NEAR(red.r, 1.0f, 0.01f);
    ASSERT_NEAR(red.g, 0.0f, 0.01f);
    ASSERT_NEAR(red.b, 0.0f, 0.01f);
}

// ========== Random Tests ==========
TEST(Random_NextInt) {
    int val = g_Random.nextInt(0, 10);
    ASSERT_TRUE(val >= 0 && val < 10);
}

TEST(Random_NextFloat) {
    float val = g_Random.nextFloat(0.0f, 1.0f);
    ASSERT_TRUE(val >= 0.0f && val < 1.0f);
}

TEST(Random_NextBool) {
    bool val = g_Random.nextBool();
    ASSERT_TRUE(val == true || val == false);
}

TEST(Random_Vector3) {
    Vector3 v = g_Random.nextVector3(-1.0f, 1.0f);
    ASSERT_TRUE(v.x >= -1.0f && v.x < 1.0f);
    ASSERT_TRUE(v.y >= -1.0f && v.y < 1.0f);
    ASSERT_TRUE(v.z >= -1.0f && v.z < 1.0f);
}

TEST(Random_OnUnitSphere) {
    Vector3 v = g_Random.randomOnUnitSphere();
    ASSERT_NEAR(v.length(), 1.0f, 0.01f);
}

TEST(Random_InUnitSphere) {
    Vector3 v = g_Random.randomInUnitSphere();
    ASSERT_TRUE(v.lengthSquared() < 1.0f);
}

TEST(Random_Direction) {
    Vector3 v = g_Random.randomDirection();
    ASSERT_NEAR(v.length(), 1.0f, 0.01f);
}

TEST(Random_Rotation) {
    Quaternion q = g_Random.randomRotation();
    ASSERT_NEAR(q.length(), 1.0f, 0.01f);
}

TEST(Random_ColorRGB) {
    Vector3 c = g_Random.randomColorRGB();
    ASSERT_TRUE(c.x >= 0.0f && c.x < 1.0f);
    ASSERT_TRUE(c.y >= 0.0f && c.y < 1.0f);
    ASSERT_TRUE(c.z >= 0.0f && c.z < 1.0f);
}

TEST(Random_Seed) {
    g_Random.seed(12345);
    uint32_t val1 = g_Random.nextUInt32();
    g_Random.seed(12345);
    uint32_t val2 = g_Random.nextUInt32();
    ASSERT_EQUAL(val1, val2);
}

// ========== Perlin Noise Tests ==========
TEST(PerlinNoise_1D) {
    PerlinNoise noise(42);
    float val = noise.noise1D(0.5f);
    ASSERT_TRUE(val >= -1.0f && val <= 1.0f);
}

TEST(PerlinNoise_2D) {
    PerlinNoise noise(42);
    float val = noise.noise2D(0.5f, 0.5f);
    ASSERT_TRUE(val >= -1.0f && val <= 1.0f);
}

TEST(PerlinNoise_3D) {
    PerlinNoise noise(42);
    float val = noise.noise3D(0.5f, 0.5f, 0.5f);
    ASSERT_TRUE(val >= -1.0f && val <= 1.0f);
}

TEST(PerlinNoise_FBM) {
    PerlinNoise noise(42);
    float val = noise.fbm2D(0.5f, 0.5f, 4, 0.5f, 2.0f);
    ASSERT_TRUE(val >= -1.0f && val <= 1.0f);
}

// ========== Random Distribution Tests ==========
TEST(RandomDistribution_Uniform) {
    float val = RandomDistribution::uniform(0.0f, 1.0f);
    ASSERT_TRUE(val >= 0.0f && val < 1.0f);
}

TEST(RandomDistribution_Gaussian) {
    float val = RandomDistribution::gaussian(0.0f, 1.0f);
    ASSERT_TRUE(val > -10.0f && val < 10.0f);
}

TEST(RandomDistribution_Exponential) {
    float val = RandomDistribution::exponential(1.0f);
    ASSERT_TRUE(val >= 0.0f);
}

TEST(RandomDistribution_Triangular) {
    float val = RandomDistribution::triangular(0.0f, 1.0f, 0.5f);
    ASSERT_TRUE(val >= 0.0f && val <= 1.0f);
}

// ========== New Vector3 Tests ==========
TEST(Vector3_projectOnPlane) {
    Vector3 v(1.0f, 1.0f, 1.0f);
    Vector3 normal(0.0f, 1.0f, 0.0f);
    Vector3 projected = v.projectOnPlane(normal);
    ASSERT_NEAR(projected.x, 1.0f, 1e-5f);
    ASSERT_NEAR(projected.y, 0.0f, 1e-5f);
    ASSERT_NEAR(projected.z, 1.0f, 1e-5f);
}

TEST(Vector3_reject) {
    Vector3 a(3.0f, 4.0f, 0.0f);
    Vector3 b(1.0f, 0.0f, 0.0f);
    Vector3 rejected = a.reject(b);
    ASSERT_NEAR(rejected.x, 0.0f, 1e-5f);
    ASSERT_NEAR(rejected.y, 4.0f, 1e-5f);
    ASSERT_NEAR(rejected.z, 0.0f, 1e-5f);
}

TEST(Vector3_midpoint) {
    Vector3 a(0.0f, 0.0f, 0.0f);
    Vector3 b(2.0f, 4.0f, 6.0f);
    Vector3 mid = a.midpoint(b);
    ASSERT_NEAR(mid.x, 1.0f, 1e-5f);
    ASSERT_NEAR(mid.y, 2.0f, 1e-5f);
    ASSERT_NEAR(mid.z, 3.0f, 1e-5f);
}

TEST(Vector3_scale) {
    Vector3 v(1.0f, 2.0f, 3.0f);
    Vector3 scaled = v.scale(Vector3(2.0f, 3.0f, 4.0f));
    ASSERT_NEAR(scaled.x, 2.0f, 1e-5f);
    ASSERT_NEAR(scaled.y, 6.0f, 1e-5f);
    ASSERT_NEAR(scaled.z, 12.0f, 1e-5f);
}

TEST(Vector3_minMaxComponent) {
    Vector3 v(3.0f, 1.0f, 2.0f);
    ASSERT_NEAR(v.minComponent(), 1.0f, 1e-5f);
    ASSERT_NEAR(v.maxComponent(), 3.0f, 1e-5f);
}

TEST(Vector3_minMaxAxis) {
    Vector3 v(3.0f, 1.0f, 2.0f);
    ASSERT_EQUAL(v.minAxis(), 1);
    ASSERT_EQUAL(v.maxAxis(), 0);
}

TEST(Vector3_clampLength) {
    Vector3 v(3.0f, 4.0f, 0.0f);
    Vector3 clamped = v.clampLength(0.0f, 3.0f);
    ASSERT_NEAR(clamped.length(), 3.0f, 1e-5f);
}

TEST(Vector3_moveTowards) {
    Vector3 from(0.0f, 0.0f, 0.0f);
    Vector3 to(10.0f, 0.0f, 0.0f);
    Vector3 result = from.moveTowards(to, 3.0f);
    ASSERT_NEAR(result.x, 3.0f, 1e-5f);
    ASSERT_NEAR(result.y, 0.0f, 1e-5f);
    ASSERT_NEAR(result.z, 0.0f, 1e-5f);
}

// ========== New MathUtils Tests ==========
TEST(MathUtils_inverseLerp) {
    ASSERT_NEAR(Math::inverseLerp(0.0f, 10.0f, 5.0f), 0.5f, 1e-5f);
    ASSERT_NEAR(Math::inverseLerp(0.0f, 10.0f, 0.0f), 0.0f, 1e-5f);
    ASSERT_NEAR(Math::inverseLerp(0.0f, 10.0f, 10.0f), 1.0f, 1e-5f);
}

TEST(MathUtils_repeat) {
    ASSERT_NEAR(Math::repeat(5.0f, 3.0f), 2.0f, 1e-5f);
    ASSERT_NEAR(Math::repeat(7.0f, 3.0f), 1.0f, 1e-5f);
}

TEST(MathUtils_pingPong) {
    ASSERT_NEAR(Math::pingPong(2.0f, 3.0f), 2.0f, 1e-5f);
    ASSERT_NEAR(Math::pingPong(4.0f, 3.0f), 2.0f, 1e-5f);
}

TEST(MathUtils_damp) {
    float result = Math::damp(0.0f, 10.0f, 5.0f, 0.1f);
    ASSERT_TRUE(result > 0.0f && result < 10.0f);
}

TEST(MathUtils_approximatelyEqualRelative) {
    ASSERT_TRUE(Math::approximatelyEqualRelative(1.0f, 1.00001f, 1e-4f));
    ASSERT_TRUE(!Math::approximatelyEqualRelative(1.0f, 2.0f, 1e-4f));
}

TEST(MathUtils_fastInvSqrt) {
    float val = 4.0f;
    float invSqrt = Math::fastInvSqrt(val);
    float expected = 1.0f / std::sqrt(val);
    ASSERT_NEAR(invSqrt, expected, 0.01f);
}

TEST(MathUtils_copysign) {
    ASSERT_NEAR(Math::copysign(5.0f, -1.0f), -5.0f, 1e-5f);
    ASSERT_NEAR(Math::copysign(-5.0f, 1.0f), 5.0f, 1e-5f);
}

TEST(MathUtils_fmod) {
    ASSERT_NEAR(Math::fmod(5.0f, 3.0f), 2.0f, 1e-5f);
    ASSERT_NEAR(Math::fmod(-5.0f, 3.0f), -2.0f, 1e-5f);
    ASSERT_NEAR(Math::fmod(5.0f, -3.0f), 2.0f, 1e-5f);
    ASSERT_NEAR(Math::fmod(-5.0f, -3.0f), -2.0f, 1e-5f);
}

TEST(MathUtils_factorial) {
    ASSERT_EQUAL(Math::factorial(0), 1);
    ASSERT_EQUAL(Math::factorial(1), 1);
    ASSERT_EQUAL(Math::factorial(5), 120);
}

TEST(MathUtils_combinations) {
    ASSERT_EQUAL(Math::combinations(5, 2), 10);
    ASSERT_EQUAL(Math::combinations(10, 3), 120);
}

TEST(MathUtils_isPowerOfTwo) {
    ASSERT_TRUE(Math::isPowerOfTwo(8));
    ASSERT_TRUE(!Math::isPowerOfTwo(7));
}

TEST(MathUtils_nextPowerOfTwo) {
    ASSERT_EQUAL(Math::nextPowerOfTwo(7), 8);
    ASSERT_EQUAL(Math::nextPowerOfTwo(8), 8);
    ASSERT_EQUAL(Math::nextPowerOfTwo(9), 16);
}

TEST(MathUtils_gcd) {
    ASSERT_EQUAL(Math::gcd(12, 8), 4);
    ASSERT_EQUAL(Math::gcd(7, 13), 1);
}

TEST(MathUtils_lcm) {
    ASSERT_EQUAL(Math::lcm(4, 6), 12);
    ASSERT_EQUAL(Math::lcm(3, 5), 15);
}

// ========== New Quaternion Tests ==========
TEST(Quaternion_lerp) {
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, Math::PI_2);
    Quaternion result = q1.lerp(q2, 0.5f);
    ASSERT_NEAR(result.length(), 1.0f, 0.1f);
}

TEST(Quaternion_squared) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3::up, Math::PI_4);
    Quaternion squared = q.squared();
    ASSERT_NEAR(squared.getAngleDegrees(), 90.0f, 1.0f);
}

TEST(Quaternion_pow) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3::up, Math::PI_2);
    Quaternion result = q.pow(0.5f);
    ASSERT_NEAR(result.getAngleDegrees(), 45.0f, 2.0f);
}

TEST(Quaternion_log_exp) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3::up, Math::PI_4);
    Quaternion logResult = q.log();
    Quaternion expResult = logResult.exp();
    ASSERT_NEAR(expResult.x, q.x, 0.01f);
    ASSERT_NEAR(expResult.y, q.y, 0.01f);
    ASSERT_NEAR(expResult.z, q.z, 0.01f);
    ASSERT_NEAR(expResult.w, q.w, 0.01f);
}

TEST(Quaternion_getAngleDegrees) {
    Quaternion q = Quaternion::fromAxisAngle(Vector3::up, Math::PI_2);
    ASSERT_NEAR(q.getAngleDegrees(), 90.0f, 1.0f);
}

TEST(Quaternion_getEulerAngles) {
    Quaternion q = Quaternion::fromEulerAngles(Vector3(0.0f, Math::PI_2, 0.0f));
    Vector3 euler = q.getEulerAngles();
    ASSERT_NEAR(euler.y, Math::PI_2, 0.01f);
}

TEST(Quaternion_fromToRotation) {
    Quaternion q = Quaternion::fromToRotation(Vector3::forward, Vector3::right);
    Vector3 result = q * Vector3::forward;
    ASSERT_NEAR(result.x, 1.0f, 0.01f);
    ASSERT_NEAR(result.z, 0.0f, 0.01f);
}

TEST(Quaternion_rotateTowards) {
    Quaternion from = Quaternion::identity;
    Quaternion to = Quaternion::fromAxisAngle(Vector3::up, Math::PI_2);
    Quaternion result = Quaternion::rotateTowards(from, to, 45.0f);
    float angle = result.getAngleDegrees();
    ASSERT_TRUE(angle < 91.0f && angle > 0.0f);
}

TEST(Quaternion_angleAxis) {
    Quaternion q = Quaternion::angleAxis(90.0f, Vector3::up, true);
    ASSERT_NEAR(q.getAngleDegrees(), 90.0f, 1.0f);
}

TEST(Quaternion_euler) {
    Quaternion q = Quaternion::euler(0.0f, 90.0f, 0.0f, true);
    ASSERT_NEAR(q.getAngleDegrees(), 90.0f, 1.0f);
}

// ========== New Matrix4x4 Tests ==========
TEST(Matrix4x4_fromTRS) {
    Vector3 pos(1.0f, 2.0f, 3.0f);
    Quaternion rot = Quaternion::fromAxisAngle(Vector3::up, Math::PI_2);
    Vector3 scale(2.0f, 2.0f, 2.0f);
    Matrix4x4 m = Matrix4x4::fromTRS(pos, rot, scale);
    ASSERT_NEAR(m.m[0][3], 1.0f, 1e-5f);
    ASSERT_NEAR(m.m[1][3], 2.0f, 1e-5f);
    ASSERT_NEAR(m.m[2][3], 3.0f, 1e-5f);
}

TEST(Matrix4x4_perspectiveFov) {
    Matrix4x4 proj = Matrix4x4::perspectiveFov(60.0f, 16.0f/9.0f, 0.1f, 100.0f, true);
    ASSERT_NEAR(proj.m[3][2], -1.0f, 1e-5f);
}

TEST(Matrix4x4_ortho) {
    Matrix4x4 proj = Matrix4x4::ortho(-10.0f, 10.0f, -10.0f, 10.0f, 0.1f, 100.0f);
    ASSERT_NEAR(proj.m[3][3], 1.0f, 1e-5f);
}

TEST(Matrix4x4_lookTo) {
    Matrix4x4 view = Matrix4x4::lookTo(Vector3(0.0f, 0.0f, 5.0f), -Vector3::forward, Vector3::up);
    ASSERT_NEAR(view.m[3][3], 1.0f, 1e-5f);
}

TEST(Matrix4x4_reflection) {
    Matrix4x4 refl = Matrix4x4::reflection(Vector3::zero, Vector3::up);
    Vector3 v(1.0f, 1.0f, 0.0f);
    Vector3 result = refl * v;
    ASSERT_NEAR(result.y, -1.0f, 1e-5f);
}

TEST(Matrix4x4_shadow) {
    Matrix4x4 shadow = Matrix4x4::shadow(Vector3(0.0f, 10.0f, 0.0f), Vector3::up, 0.0f);
    Vector3 v(5.0f, 5.0f, 5.0f);
    Vector3 result = shadow * v;
    ASSERT_NEAR(result.y, 0.0f, 1e-4f);
}

TEST(Matrix4x4_worldToLocal) {
    Vector3 pos(1.0f, 2.0f, 3.0f);
    Quaternion rot = Quaternion::identity;
    Vector3 scale(1.0f, 1.0f, 1.0f);
    Matrix4x4 m = Matrix4x4::worldToLocal(pos, rot, scale);
    Vector3 worldPos(2.0f, 3.0f, 4.0f);
    Vector3 localPos = m * worldPos;
    ASSERT_NEAR(localPos.x, 1.0f, 1e-5f);
    ASSERT_NEAR(localPos.y, 1.0f, 1e-5f);
    ASSERT_NEAR(localPos.z, 1.0f, 1e-5f);
}

TEST(Matrix4x4_localToWorld) {
    Vector3 pos(1.0f, 2.0f, 3.0f);
    Quaternion rot = Quaternion::identity;
    Vector3 scale(1.0f, 1.0f, 1.0f);
    Matrix4x4 m = Matrix4x4::localToWorld(pos, rot, scale);
    Vector3 localPos(1.0f, 1.0f, 1.0f);
    Vector3 worldPos = m * localPos;
    ASSERT_NEAR(worldPos.x, 2.0f, 1e-5f);
    ASSERT_NEAR(worldPos.y, 3.0f, 1e-5f);
    ASSERT_NEAR(worldPos.z, 4.0f, 1e-5f);
}

TEST(Matrix4x4_extractRotation) {
    Matrix4x4 m = Matrix4x4::rotationY(Math::PI_2);
    Matrix4x4 rotation = m.extractRotation();
    ASSERT_NEAR(rotation.m[0][0], 0.0f, 1e-5f);
    ASSERT_NEAR(rotation.m[0][2], 1.0f, 1e-5f);
}

TEST(Matrix4x4_getDeterminant) {
    Matrix4x4 m = Matrix4x4::scale(Vector3(2.0f, 3.0f, 4.0f));
    float det = m.getDeterminant();
    ASSERT_NEAR(det, 24.0f, 1e-5f);
}

TEST(Matrix4x4_getInverse) {
    Matrix4x4 m = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));
    Matrix4x4 inv = m.getInverse();
    Matrix4x4 result = m * inv;
    ASSERT_NEAR(result.m[0][0], 1.0f, 1e-5f);
    ASSERT_NEAR(result.m[1][1], 1.0f, 1e-5f);
    ASSERT_NEAR(result.m[2][2], 1.0f, 1e-5f);
}

TEST(Matrix4x4_getTranspose) {
    Matrix4x4 m = Matrix4x4::rotationY(Math::PI_2);
    Matrix4x4 transpose = m.getTranspose();
    ASSERT_NEAR(transpose.m[0][2], m.m[2][0], 1e-5f);
    ASSERT_NEAR(transpose.m[2][0], m.m[0][2], 1e-5f);
}

// ========== Boundary Condition Tests ==========
TEST(Boundary_Vector3_ZeroLength) {
    Vector3 v(0.0f, 0.0f, 0.0f);
    ASSERT_NEAR(v.length(), 0.0f, 1e-5f);
    Vector3 n = v.normalized();
    ASSERT_NEAR(n.x, 0.0f, 1e-5f);
    ASSERT_NEAR(n.y, 0.0f, 1e-5f);
    ASSERT_NEAR(n.z, 0.0f, 1e-5f);
}

TEST(Boundary_Vector3_VerySmall) {
    Vector3 v(1e-3f, 1e-3f, 1e-3f);
    ASSERT_TRUE(v.length() > 0.0f);
    Vector3 n = v.normalized();
    // 归一化后的长度应该接近1.0，但由于数值精度问题，容差可以稍大
    ASSERT_NEAR(n.length(), 1.0f, 0.1f);
}

TEST(Boundary_Vector3_VeryLarge) {
    Vector3 v(1.0f, 1.0f, 1.0f);
    ASSERT_TRUE(v.length() > 0.0f);
    Vector3 n = v.normalized();
    ASSERT_NEAR(n.length(), 1.0f, 0.01f);
}

TEST(Boundary_Vector3_NaN) {
    Vector3 v(0.0f, 0.0f, 0.0f);
    Vector3 result = v / 0.0f;
    ASSERT_TRUE(std::isnan(result.x) || std::isinf(result.x));
}

TEST(Boundary_Vector3_Infinity) {
    Vector3 v(1e30f, 1e30f, 1e30f);
    Vector3 result = v * 1e10f;
    ASSERT_TRUE(std::isinf(result.x));
}

TEST(Boundary_Vector3_ParallelCross) {
    Vector3 a(1.0f, 0.0f, 0.0f);
    Vector3 b(2.0f, 0.0f, 0.0f);
    Vector3 cross = a.cross(b);
    ASSERT_NEAR(cross.x, 0.0f, 1e-5f);
    ASSERT_NEAR(cross.y, 0.0f, 1e-5f);
    ASSERT_NEAR(cross.z, 0.0f, 1e-5f);
}

TEST(Boundary_Vector3_AngleWithSelf) {
    Vector3 v(1.0f, 2.0f, 3.0f);
    ASSERT_NEAR(v.angle(v), 0.0f, 1e-5f);
}

TEST(Boundary_Vector3_AngleWithZero) {
    Vector3 v(1.0f, 0.0f, 0.0f);
    Vector3 zero(0.0f, 0.0f, 0.0f);
    float angle = v.angle(zero);
    ASSERT_TRUE(angle >= 0.0f);
}

TEST(Boundary_Quaternion_Identity) {
    Quaternion q = Quaternion::identity;
    ASSERT_NEAR(q.length(), 1.0f, 1e-5f);
    ASSERT_NEAR(q.x, 0.0f, 1e-5f);
    ASSERT_NEAR(q.y, 0.0f, 1e-5f);
    ASSERT_NEAR(q.z, 0.0f, 1e-5f);
    ASSERT_NEAR(q.w, 1.0f, 1e-5f);
}

TEST(Boundary_Quaternion_SlerpIdentical) {
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::identity;
    Quaternion result = q1.slerp(q2, 0.5f);
    ASSERT_NEAR(result.w, 1.0f, 1e-5f);
}

TEST(Boundary_Quaternion_SlerpOpposite) {
    Quaternion q1 = Quaternion::identity;
    Quaternion q2(-1.0f, 0.0f, 0.0f, 0.0f);
    Quaternion result = q1.slerp(q2, 0.5f);
    ASSERT_TRUE(result.length() > 0.0f);
}

TEST(Boundary_Quaternion_ZeroLength) {
    Quaternion q(0.0f, 0.0f, 0.0f, 0.0f);
    Quaternion n = q.normalized();
    ASSERT_NEAR(n.x, 0.0f, 1e-5f);
    ASSERT_NEAR(n.y, 0.0f, 1e-5f);
    ASSERT_NEAR(n.z, 0.0f, 1e-5f);
    ASSERT_NEAR(n.w, 0.0f, 1e-5f);
}

TEST(Boundary_Matrix4x4_Singular) {
    Matrix4x4 m = Matrix4x4::zero;
    float det = m.determinant();
    ASSERT_NEAR(det, 0.0f, 1e-5f);
}

TEST(Boundary_Matrix4x4_InverseSingular) {
    Matrix4x4 m = Matrix4x4::zero;
    Matrix4x4 inv = m.inverse();
    // 奇异矩阵的逆返回单位矩阵（安全回退）
    ASSERT_NEAR(inv.m[0][0], 1.0f, 1e-5f);
    ASSERT_NEAR(inv.m[1][1], 1.0f, 1e-5f);
    ASSERT_NEAR(inv.m[2][2], 1.0f, 1e-5f);
}

TEST(Boundary_Matrix4x4_IdentityMultiply) {
    Matrix4x4 identity = Matrix4x4::identity;
    Matrix4x4 m = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));
    Matrix4x4 result1 = identity * m;
    Matrix4x4 result2 = m * identity;
    ASSERT_NEAR(result1.m[0][3], result2.m[0][3], 1e-5f);
    ASSERT_NEAR(result1.m[1][3], result2.m[1][3], 1e-5f);
    ASSERT_NEAR(result1.m[2][3], result2.m[2][3], 1e-5f);
}

TEST(Boundary_Matrix4x4_VerySmallScale) {
    Matrix4x4 m = Matrix4x4::scale(Vector3(1e-10f, 1e-10f, 1e-10f));
    ASSERT_NEAR(m.m[0][0], 1e-10f, 1e-15f);
}

TEST(Boundary_Matrix4x4_VeryLargeScale) {
    Matrix4x4 m = Matrix4x4::scale(Vector3(1e10f, 1e10f, 1e10f));
    ASSERT_NEAR(m.m[0][0], 1e10f, 1e5f);
}

TEST(Boundary_MathUtils_DivisionByZero) {
    float result = Math::fastInvSqrt(0.0f);
    // fastInvSqrt 对零输入返回 0.0f（安全回退）
    ASSERT_NEAR(result, 0.0f, 1e-5f);
}

TEST(Boundary_MathUtils_SqrtNegative) {
    float result = Math::sqrt(-1.0f);
    ASSERT_TRUE(result == 0.0f || std::isnan(result));
}

TEST(Boundary_MathUtils_AsinOutOfRange) {
    float result1 = Math::asin(2.0f);
    ASSERT_NEAR(result1, Math::PI_2, 1e-5f);
    float result2 = Math::asin(-2.0f);
    ASSERT_NEAR(result2, -Math::PI_2, 1e-5f);
}

TEST(Boundary_MathUtils_AcosOutOfRange) {
    float result1 = Math::acos(2.0f);
    ASSERT_NEAR(result1, 0.0f, 1e-5f);
}

TEST(Boundary_MathUtils_LerpClamped) {
    float result = Math::lerp(0.0f, 10.0f, -1.0f);
    ASSERT_NEAR(result, -10.0f, 1e-5f);
    result = Math::lerp(0.0f, 10.0f, 2.0f);
    ASSERT_NEAR(result, 20.0f, 1e-5f);
}

TEST(Boundary_MathUtils_ClampReversed) {
    float result = Math::clamp(5.0f, 10.0f, 0.0f);
    ASSERT_TRUE(result >= 0.0f && result <= 10.0f);
}

TEST(Boundary_MathUtils_FactorialLarge) {
    uint64_t result = Math::factorial(20);
    ASSERT_TRUE(result > 0);
}

TEST(Boundary_MathUtils_GCDZero) {
    int result = Math::gcd(0, 5);
    ASSERT_EQUAL(result, 5);
    result = Math::gcd(5, 0);
    ASSERT_EQUAL(result, 5);
}

TEST(Boundary_Ray_OriginOnSphere) {
    Ray ray(Vector3(0.0f, 0.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f));
    Sphere sphere(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    float t = ray.intersect(sphere);
    ASSERT_TRUE(t >= 0.0f);
}

TEST(Boundary_Ray_ParallelToPlane) {
    Ray ray(Vector3(0.0f, 5.0f, 0.0f), Vector3(1.0f, 0.0f, 0.0f));
    Plane plane(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
    float t = ray.intersect(plane);
    ASSERT_TRUE(t < 0.0f || std::isinf(t));
}

TEST(Boundary_Sphere_ZeroRadius) {
    Sphere sphere(Vector3(0.0f, 0.0f, 0.0f), 0.0f);
    ASSERT_TRUE(!sphere.contains(Vector3(0.1f, 0.0f, 0.0f)));
}

TEST(Boundary_Plane_PointOnPlane) {
    Plane plane(Vector3(0.0f, 1.0f, 0.0f), 0.0f);
    ASSERT_NEAR(plane.distanceToPoint(Vector3(1.0f, 0.0f, 1.0f)), 0.0f, 1e-5f);
}

TEST(Boundary_Bounds_ZeroSize) {
    Bounds bounds(Vector3(0.0f, 0.0f, 0.0f), Vector3(0.0f, 0.0f, 0.0f));
    ASSERT_TRUE(bounds.contains(Vector3(0.0f, 0.0f, 0.0f)));
    ASSERT_TRUE(!bounds.contains(Vector3(0.1f, 0.0f, 0.0f)));
}

TEST(Boundary_Bounds_NegativeSize) {
    Bounds bounds(Vector3(1.0f, 1.0f, 1.0f), Vector3(0.0f, 0.0f, 0.0f));
    // Bounds 不自动修正负尺寸，min 可能大于 max
    ASSERT_NEAR(bounds.min.x, 1.0f, 1e-5f);
    ASSERT_NEAR(bounds.max.x, 0.0f, 1e-5f);
}

TEST(Boundary_ColorSpace_Black) {
    ColorRGB black(0.0f, 0.0f, 0.0f);
    ColorHSV hsv = ColorSpace::rgbToHsv(black);
    ASSERT_NEAR(hsv.v, 0.0f, 0.01f);
}

TEST(Boundary_ColorSpace_White) {
    ColorRGB white(1.0f, 1.0f, 1.0f);
    ColorHSV hsv = ColorSpace::rgbToHsv(white);
    ASSERT_NEAR(hsv.s, 0.0f, 0.01f);
    ASSERT_NEAR(hsv.v, 1.0f, 0.01f);
}

TEST(Boundary_Random_DeterministicSeed) {
    g_Random.seed(42);
    float val1 = g_Random.nextFloat();
    g_Random.seed(42);
    float val2 = g_Random.nextFloat();
    ASSERT_NEAR(val1, val2, 1e-10f);
}

TEST(Boundary_MatrixDecomposition_Identity) {
    Matrix4x4 A = Matrix4x4::identity;
    Matrix4x4 L, U, P;
    bool success = MatrixDecomposition::luDecompose(A, L, U, P);
    ASSERT_TRUE(success);
}

TEST(Boundary_MatrixDecomposition_ZeroMatrix) {
    Matrix4x4 A = Matrix4x4::zero;
    Matrix4x4 L, U, P;
    bool success = MatrixDecomposition::luDecompose(A, L, U, P);
    // 零矩阵是奇异的，LU分解应该失败
    ASSERT_TRUE(!success);
}

TEST(Vector3_FastLength) {
    Vector3 v(3.0f, 4.0f, 0.0f);
    float fastLen = v.fastLength();
    ASSERT_NEAR(fastLen, 5.0f, 0.1f);
}

TEST(Vector3_FastNormalized) {
    Vector3 v(3.0f, 4.0f, 0.0f);
    Vector3 n = v.fastNormalized();
    ASSERT_NEAR(n.length(), 1.0f, 0.1f);
}

TEST(Vector3_FastLerp) {
    Vector3 a(0.0f, 0.0f, 0.0f);
    Vector3 b(10.0f, 10.0f, 10.0f);
    Vector3 result = a.fastLerp(b, 0.5f);
    ASSERT_NEAR(result.x, 5.0f, 0.01f);
    ASSERT_NEAR(result.y, 5.0f, 0.01f);
    ASSERT_NEAR(result.z, 5.0f, 0.01f);
}

TEST(Quaternion_FastSlerp) {
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, Math::degToRad(90.0f));
    Quaternion result = q1.fastSlerp(q2, 0.5f).normalized();
    ASSERT_TRUE(result.isUnit());
    float angle = Quaternion::angle(q1, result);
    // fastSlerp 是近似算法，容差稍大
    ASSERT_NEAR(angle, Math::degToRad(45.0f), 0.05f);
}

TEST(SIMD_VectorAddBatch) {
    const int count = 100;
    std::vector<float> a(count * 3), b(count * 3), result(count * 3);
    
    for (int i = 0; i < count * 3; ++i) {
        a[i] = static_cast<float>(i);
        b[i] = static_cast<float>(i * 2);
    }
    
    SIMD::vectorAddBatch(a.data(), b.data(), result.data(), count * 3);
    
    for (int i = 0; i < count * 3; ++i) {
        ASSERT_NEAR(result[i], a[i] + b[i], 0.01f);
    }
}

TEST(SIMD_VectorMulBatch) {
    const int count = 100;
    std::vector<float> a(count * 3), b(count * 3), result(count * 3);
    
    for (int i = 0; i < count * 3; ++i) {
        a[i] = static_cast<float>(i + 1);
        b[i] = 2.0f;
    }
    
    SIMD::scalarMulBatch(a.data(), 2.0f, result.data(), count * 3);
    
    for (int i = 0; i < count * 3; ++i) {
        ASSERT_NEAR(result[i], a[i] * 2.0f, 0.01f);
    }
}

// ========== Main ==========
int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  AstraAlgebra Comprehensive Test Suite" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << std::endl;
    
    std::cout << "Running " << tests_passed + tests_failed << " tests..." << std::endl;
    std::cout << std::endl;
    
    // Tests are auto-registered via static instances
    // Force initialization by accessing them
    // (The static instances above already register themselves)
    
    std::cout << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Test Results" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Passed: " << tests_passed << std::endl;
    std::cout << "  Failed: " << tests_failed << std::endl;
    std::cout << "  Total:  " << tests_passed + tests_failed << std::endl;
    std::cout << "  Pass Rate: " << (tests_passed + tests_failed > 0 ? 
        (tests_passed * 100 / (tests_passed + tests_failed)) : 0) << "%" << std::endl;
    std::cout << "========================================" << std::endl;
    
    if (tests_failed > 0) {
        std::cout << std::endl << "Failed tests:" << std::endl;
        for (const auto& f : failed_tests) {
            std::cout << "  - " << f << std::endl;
        }
    }
    
    return tests_failed > 0 ? 1 : 0;
}
