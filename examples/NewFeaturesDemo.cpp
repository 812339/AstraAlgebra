#include <iostream>
#include <AstraAlgebra/AstraAlgebra.h>

using namespace AstraAlgebra;

int main() {
    std::cout << "=== AstraAlgebra 功能演示 ===" << std::endl;
    std::cout << std::endl;

    std::cout << "--- 向量新增功能 ---" << std::endl;
    Vector3 v1(3.0f, 4.0f, 0.0f);
    std::cout << "向量: " << v1.x << ", " << v1.y << ", " << v1.z << std::endl;
    std::cout << "长度 (magnitude): " << v1.magnitude() << std::endl;
    std::cout << "长度平方 (magnitudeSquared): " << v1.magnitudeSquared() << std::endl;
    std::cout << "单位向量 (unit): " << v1.unit().x << ", " << v1.unit().y << ", " << v1.unit().z << std::endl;
    std::cout << "最小分量: " << v1.minComponent() << std::endl;
    std::cout << "最大分量: " << v1.maxComponent() << std::endl;
    std::cout << "最小轴: " << v1.minAxis() << std::endl;
    std::cout << "最大轴: " << v1.maxAxis() << std::endl;

    Vector3 v2(1.0f, 2.0f, 3.0f);
    Vector3 v3(4.0f, 5.0f, 6.0f);
    std::cout << "中点: " << v2.midpoint(v3).x << ", " << v2.midpoint(v3).y << ", " << v2.midpoint(v3).z << std::endl;
    std::cout << "缩放: " << v2.scale(Vector3(2.0f, 2.0f, 2.0f)).x << ", " << v2.scale(Vector3(2.0f, 2.0f, 2.0f)).y << std::endl;

    Vector3 target(10.0f, 0.0f, 0.0f);
    Vector3 from(0.0f, 0.0f, 0.0f);
    Vector3 result = from.moveTowards(target, 3.0f);
    std::cout << "moveTowards: " << result.x << ", " << result.y << ", " << result.z << std::endl;

    std::cout << std::endl;
    std::cout << "--- MathUtils 新增功能 ---" << std::endl;
    std::cout << "inverseLerp(0, 10, 5): " << Math::inverseLerp(0.0f, 10.0f, 5.0f) << std::endl;
    std::cout << "repeat(7, 3): " << Math::repeat(7.0f, 3.0f) << std::endl;
    std::cout << "pingPong(4, 3): " << Math::pingPong(4.0f, 3.0f) << std::endl;
    std::cout << "damp(0, 10, 5, 0.1): " << Math::damp(0.0f, 10.0f, 5.0f, 0.1f) << std::endl;
    std::cout << "factorial(5): " << Math::factorial(5) << std::endl;
    std::cout << "combinations(5, 2): " << Math::combinations(5, 2) << std::endl;
    std::cout << "isPowerOfTwo(8): " << Math::isPowerOfTwo(8) << std::endl;
    std::cout << "nextPowerOfTwo(7): " << Math::nextPowerOfTwo(7) << std::endl;
    std::cout << "gcd(12, 8): " << Math::gcd(12, 8) << std::endl;
    std::cout << "lcm(4, 6): " << Math::lcm(4, 6) << std::endl;

    std::cout << std::endl;
    std::cout << "--- 四元数新增功能 ---" << std::endl;
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, Math::PI_2);
    Quaternion lerpResult = q1.lerp(q2, 0.5f);
    std::cout << "lerp 角度: " << lerpResult.getAngleDegrees() << std::endl;
    std::cout << "squared 角度: " << q1.squared().getAngleDegrees() << std::endl;

    Quaternion fromTo = Quaternion::fromToRotation(Vector3::forward, Vector3::right);
    Vector3 rotated = fromTo * Vector3::forward;
    std::cout << "fromToRotation 结果: " << rotated.x << ", " << rotated.y << ", " << rotated.z << std::endl;

    std::cout << std::endl;
    std::cout << "--- 矩阵新增功能 ---" << std::endl;
    Matrix4x4 trs = Matrix4x4::fromTRS(
        Vector3(1.0f, 2.0f, 3.0f),
        Quaternion::fromAxisAngle(Vector3::up, Math::PI_4),
        Vector3(2.0f, 2.0f, 2.0f)
    );
    std::cout << "TRS 矩阵平移: " << trs.m[0][3] << ", " << trs.m[1][3] << ", " << trs.m[2][3] << std::endl;

    Matrix4x4 perspective = Matrix4x4::perspectiveFov(60.0f, 16.0f/9.0f, 0.1f, 100.0f, true);
    std::cout << "透视投影 m[3][2]: " << perspective.m[3][2] << std::endl;

    Matrix4x4 ortho = Matrix4x4::ortho(-10.0f, 10.0f, -10.0f, 10.0f, 0.1f, 100.0f);
    std::cout << "正交投影 m[3][3]: " << ortho.m[3][3] << std::endl;

    Matrix4x4 scale = Matrix4x4::scale(Vector3(2.0f, 3.0f, 4.0f));
    std::cout << "行列式: " << scale.getDeterminant() << std::endl;
    std::cout << "迹: " << scale.trace() << std::endl;

    std::cout << std::endl;
    std::cout << "=== 演示完成 ===" << std::endl;
    return 0;
}
