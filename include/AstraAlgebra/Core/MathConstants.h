// MathConstants - 数学常量，PI、E那些，省得自己写

#pragma once

#include <limits>

namespace AstraAlgebra {

namespace Math {

// 圆周率
static constexpr float PI = 3.14159265358979323846f;
static constexpr float PI_2 = PI * 0.5f;      // π/2
static constexpr float PI_3 = PI / 3.0f;      // π/3
static constexpr float PI_4 = PI * 0.25f;     // π/4
static constexpr float PI_6 = PI / 6.0f;      // π/6
static constexpr float TWO_PI = PI * 2.0f;    // 2π
static constexpr float THREE_PI_2 = PI * 1.5f; // 3π/2

// 自然常数
static constexpr float E = 2.71828182845904523536f;

// 平方根
static constexpr float SQRT_2 = 1.41421356237309504880f;  // √2
static constexpr float SQRT_3 = 1.73205080756887729352f;  // √3
static constexpr float SQRT_1_2 = 0.70710678118654752440f; // 1/√2
static constexpr float SQRT_1_3 = 0.57735026918962576450f; // 1/√3

// 度跟弧度转换
static constexpr float DEG_TO_RAD = PI / 180.0f;  // 度转弧度
static constexpr float RAD_TO_DEG = 180.0f / PI;  // 弧度转度

// 极小值，避免除以0那些坑
static constexpr float EPSILON = 1.0e-6f;
static constexpr double DOUBLE_EPSILON = 1.0e-12;
static constexpr double EPSILON_D = DOUBLE_EPSILON; // 别名

// 最大最小值
static constexpr float FLOAT_MAX = std::numeric_limits<float>::max();
static constexpr float FLOAT_MIN = std::numeric_limits<float>::min();
static constexpr double DOUBLE_MAX = std::numeric_limits<double>::max();
static constexpr double DOUBLE_MIN = std::numeric_limits<double>::min();

} // namespace Math

} // namespace AstraAlgebra
