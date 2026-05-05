// MathUtils - 数学工具函数，各种常用的数学操作都在这

#pragma once

#include "MathConstants.h"
#include <cmath>
#include <algorithm>

namespace AstraAlgebra {

// Math命名空间，装各种数学工具函数
namespace Math {

// 内部实现细节，constexpr版本的数学函数，编译期就能算
namespace detail {
    // sqrt，用牛顿迭代法算，10次迭代够了
    constexpr float sqrt_float(float x) {
        if (x <= 0.0f) return 0.0f;
        float guess = x;
        for (int i = 0; i < 10; ++i) {
            guess = 0.5f * (guess + x / guess);
        }
        return guess;
    }
    
    constexpr double sqrt_double(double x) {
        if (x <= 0.0) return 0.0;
        double guess = x;
        for (int i = 0; i < 10; ++i) {
            guess = 0.5 * (guess + x / guess);
        }
        return guess;
    }
    
    // floor，向下取整
    constexpr float floor_float(float x) {
        int i = static_cast<int>(x);
        return (x < 0 && x != static_cast<float>(i)) ? static_cast<float>(i - 1) : static_cast<float>(i);
    }
    
    constexpr double floor_double(double x) {
        int i = static_cast<int>(x);
        return (x < 0 && x != static_cast<double>(i)) ? static_cast<double>(i - 1) : static_cast<double>(i);
    }
    
    // ceil，向上取整
    constexpr float ceil_float(float x) {
        int i = static_cast<int>(x);
        return (x > 0 && x != static_cast<float>(i)) ? static_cast<float>(i + 1) : static_cast<float>(i);
    }
    
    constexpr double ceil_double(double x) {
        int i = static_cast<int>(x);
        return (x > 0 && x != static_cast<double>(i)) ? static_cast<double>(i + 1) : static_cast<double>(i);
    }
    
    // round，四舍五入
    constexpr float round_float(float x) {
        return (x >= 0.0f) ? floor_float(x + 0.5f) : ceil_float(x - 0.5f);
    }
    
    constexpr double round_double(double x) {
        return (x >= 0.0) ? floor_double(x + 0.5) : ceil_double(x - 0.5);
    }
    
    // sin，用泰勒级数近似，编译期用的，运行时还是用std::sin快
    constexpr float sin_float(float x) {
        while (x > PI) x -= TWO_PI;
        while (x < -PI) x += TWO_PI;
        
        float result = x;
        float term = x;
        float x2 = x * x;
        for (int n = 1; n <= 8; ++n) {
            term *= -x2 / (2.0f * n * (2.0f * n + 1.0f));
            result += term;
        }
        return result;
    }
    
    constexpr double sin_double(double x) {
        while (x > PI) x -= TWO_PI;
        while (x < -PI) x += TWO_PI;
        
        double result = x;
        double term = x;
        double x2 = x * x;
        for (int n = 1; n <= 10; ++n) {
            term *= -x2 / (2.0 * n * (2.0 * n + 1.0));
            result += term;
        }
        return result;
    }
    
    // cos，sin换个相位就行
    constexpr float cos_float(float x) {
        return sin_float(PI_2 - x);
    }
    
    constexpr double cos_double(double x) {
        return sin_double(PI_2 - x);
    }
    
    // asin，泰勒级数展开
    constexpr float asin_float(float x) {
        if (x >= 1.0f) return PI_2;
        if (x <= -1.0f) return -PI_2;
        
        float result = x;
        float term = x;
        float x2 = x * x;
        for (int n = 1; n <= 8; ++n) {
            term *= x2 * (2.0f * n - 1.0f) * (2.0f * n - 1.0f) / (2.0f * n * (2.0f * n + 1.0f));
            result += term;
        }
        return result;
    }
    
    constexpr double asin_double(double x) {
        if (x >= 1.0) return PI_2;
        if (x <= -1.0) return -PI_2;
        
        double result = x;
        double term = x;
        double x2 = x * x;
        for (int n = 1; n <= 10; ++n) {
            term *= x2 * (2.0 * n - 1.0) * (2.0 * n - 1.0) / (2.0 * n * (2.0 * n + 1.0));
            result += term;
        }
        return result;
    }
    
    // acos，PI/2 - asin就行
    constexpr float acos_float(float x) {
        return PI_2 - asin_float(x);
    }
    
    constexpr double acos_double(double x) {
        return PI_2 - asin_double(x);
    }
}

// 角度弧度转换，这个经常用
constexpr float degToRad(float degrees) { return degrees * DEG_TO_RAD; }
constexpr double degToRad(double degrees) { return degrees * static_cast<double>(DEG_TO_RAD); }

constexpr float radToDeg(float radians) { return radians * RAD_TO_DEG; }
constexpr double radToDeg(double radians) { return radians * static_cast<double>(RAD_TO_DEG); }

// 线性插值，游戏里用得最多
constexpr float lerp(float a, float b, float t) { return a + t * (b - a); }
constexpr double lerp(double a, double b, double t) { return a + t * (b - a); }

// 球形插值，四元数旋转用
float slerp(float a, float b, float t);
double slerp(double a, double b, double t);

// smoothstep，平滑过渡，shader里经常用
constexpr float smoothStep(float edge0, float edge1, float x) {
    float t = (x < edge0) ? 0.0f : ((x > edge1) ? 1.0f : ((x - edge0) / (edge1 - edge0)));
    return t * t * (3.0f - 2.0f * t);
}
constexpr double smoothStep(double edge0, double edge1, double x) {
    double t = (x < edge0) ? 0.0 : ((x > edge1) ? 1.0 : ((x - edge0) / (edge1 - edge0)));
    return t * t * (3.0 - 2.0 * t);
}

// smootherstep，比smoothstep更平滑，Ken Perlin搞的
constexpr float smootherStep(float edge0, float edge1, float x) {
    float t = (x < edge0) ? 0.0f : ((x > edge1) ? 1.0f : ((x - edge0) / (edge1 - edge0)));
    return t * t * t * (t * (t * 6.0f - 15.0f) + 10.0f);
}
constexpr double smootherStep(double edge0, double edge1, double x) {
    double t = (x < edge0) ? 0.0 : ((x > edge1) ? 1.0 : ((x - edge0) / (edge1 - edge0)));
    return t * t * t * (t * (t * 6.0 - 15.0) + 10.0);
}

// 随机数
float random();
double randomDouble();

float randomRange(float min, float max);
double randomRange(double min, double max);

int randomInt(int min, int max);

// 角度归一化
float normalizeAngle(float angle, bool isDegrees = false);
double normalizeAngle(double angle, bool isDegrees = false);

// 两个角度之间的最小差值
float angleDifference(float a, float b, bool isDegrees = false);
double angleDifference(double a, double b, bool isDegrees = false);

// 浮点数比较，不能直接==，要注意精度
constexpr bool approximatelyEqual(float a, float b, float epsilon = EPSILON) {
    return abs(a - b) < epsilon;
}
constexpr bool approximatelyEqual(double a, double b, double epsilon = DOUBLE_EPSILON) {
    return abs(a - b) < epsilon;
}

// 符号函数
int sign(float x);
int sign(double x);

// 取整
constexpr float floor(float x) { return detail::floor_float(x); }
constexpr double floor(double x) { return detail::floor_double(x); }

constexpr float ceil(float x) { return detail::ceil_float(x); }
constexpr double ceil(double x) { return detail::ceil_double(x); }

constexpr float round(float x) { return detail::round_float(x); }
constexpr double round(double x) { return detail::round_double(x); }

// clamp，限制在[min, max]范围内
constexpr float clamp(float value, float min, float max) {
    return (value < min) ? min : ((value > max) ? max : value);
}
constexpr double clamp(double value, double min, double max) {
    return (value < min) ? min : ((value > max) ? max : value);
}
constexpr int clamp(int value, int min, int max) {
    return (value < min) ? min : ((value > max) ? max : value);
}

// 绝对值
constexpr float abs(float x) { return x < 0 ? -x : x; }
constexpr double abs(double x) { return x < 0 ? -x : x; }
constexpr int abs(int x) { return x < 0 ? -x : x; }

// 平方根
constexpr float sqrt(float x) { return detail::sqrt_float(x); }
constexpr double sqrt(double x) { return detail::sqrt_double(x); }

// 平方
constexpr float square(float x) { return x * x; }
constexpr double square(double x) { return x * x; }

// 立方
constexpr float cube(float x) { return x * x * x; }
constexpr double cube(double x) { return x * x * x; }

// 指数
float exp(float x);
double exp(double x);

// 对数
float log(float x);
double log(double x);

// 幂运算
constexpr float pow(float base, float exponent);
constexpr double pow(double base, double exponent);

// 立方根
constexpr float cbrt(float x);
constexpr double cbrt(double x);

// 双曲函数
float sinh(float x);
double sinh(double x);

float cosh(float x);
double cosh(double x);

float tanh(float x);
double tanh(double x);

// 反双曲
float asinh(float x);
double asinh(double x);

float acosh(float x);
double acosh(double x);

float atanh(float x);
double atanh(double x);

// 高斯分布随机数，正态分布
float randomGaussian(float mean = 0.0f, float stdDev = 1.0f);
double randomGaussian(double mean = 0.0, double stdDev = 1.0f);

// 最小值最大值
constexpr float min(float a, float b) { return (a < b) ? a : b; }
constexpr double min(double a, double b) { return (a < b) ? a : b; }

constexpr float max(float a, float b) { return (a > b) ? a : b; }
constexpr double max(double a, double b) { return (a > b) ? a : b; }

// clamp到[0,1]
constexpr float clamp01(float value) { return clamp(value, 0.0f, 1.0f); }
constexpr double clamp01(double value) { return clamp(value, 0.0, 1.0); }

// 颜色空间转换
void rgbToHsv(float r, float g, float b, float& h, float& s, float& v);
void rgbToHsv(double r, double g, double b, double& h, double& s, double& v);

void hsvToRgb(float h, float s, float v, float& r, float& g, float& b);
void hsvToRgb(double h, double s, double v, double& r, double& g, double& b);

void rgbToHsl(float r, float g, float b, float& h, float& s, float& l);
void rgbToHsl(double r, double g, double b, double& h, double& s, double& l);

void hslToRgb(float h, float s, float l, float& r, float& g, float& b);
void hslToRgb(double h, double s, double l, double& r, double& g, double& b);

// 四元数点积
float quatDot(float x0, float y0, float z0, float w0, float x1, float y1, float z1, float w1);
double quatDot(double x0, double y0, double z0, double w0, double x1, double y1, double z1, double w1);

// Hermite插值
float hermite(float v0, float v1, float m0, float m1, float t);
double hermite(double v0, double v1, double m0, double m1, double t);

// Bezier曲线
float bezier(float p0, float p1, float p2, float p3, float t);
double bezier(double p0, double p1, double p2, double p3, double t);

// 行列式
float determinant2x2(float a, float b, float c, float d);
double determinant2x2(double a, double b, double c, double d);

float determinant3x3(float a11, float a12, float a13, float a21, float a22, float a23, float a31, float a32, float a33);
double determinant3x3(double a11, double a12, double a13, double a21, double a22, double a23, double a31, double a32, double a33);

// sin，constexpr版本编译期用，运行时用sin_rt
template<typename T>
[[nodiscard]] constexpr T sin(T angle, bool isDegrees = false) {
    T a = isDegrees ? degToRad(angle) : angle;
    if constexpr (std::is_same_v<T, float>) {
        return detail::sin_float(a);
    } else {
        return detail::sin_double(a);
    }
}
[[nodiscard]] inline float sin_rt(float angle, bool isDegrees = false) {
    float a = isDegrees ? degToRad(angle) : angle;
    return std::sin(a);
}
[[nodiscard]] inline double sin_rt(double angle, bool isDegrees = false) {
    double a = isDegrees ? degToRad(angle) : angle;
    return std::sin(a);
}

// cos
template<typename T>
[[nodiscard]] constexpr T cos(T angle, bool isDegrees = false) {
    T a = isDegrees ? degToRad(angle) : angle;
    if constexpr (std::is_same_v<T, float>) {
        return detail::cos_float(a);
    } else {
        return detail::cos_double(a);
    }
}
[[nodiscard]] inline float cos_rt(float angle, bool isDegrees = false) {
    float a = isDegrees ? degToRad(angle) : angle;
    return std::cos(a);
}
[[nodiscard]] inline double cos_rt(double angle, bool isDegrees = false) {
    double a = isDegrees ? degToRad(angle) : angle;
    return std::cos(a);
}

// tan
template<typename T>
[[nodiscard]] constexpr T tan(T angle, bool isDegrees = false) {
    T s = sin(angle, isDegrees);
    T c = cos(angle, isDegrees);
    return (c != T(0)) ? s / c : T(0);
}
[[nodiscard]] inline float tan_rt(float angle, bool isDegrees = false) {
    float a = isDegrees ? degToRad(angle) : angle;
    return std::tan(a);
}
[[nodiscard]] inline double tan_rt(double angle, bool isDegrees = false) {
    double a = isDegrees ? degToRad(angle) : angle;
    return std::tan(a);
}

// asin
template<typename T>
[[nodiscard]] constexpr T asin(T value, bool returnDegrees = false) {
    T result = PI_2 - detail::acos_float(value);
    return returnDegrees ? radToDeg(result) : result;
}
[[nodiscard]] inline float asin_rt(float value, bool returnDegrees = false) {
    float result = std::asin(value);
    return returnDegrees ? radToDeg(result) : result;
}
[[nodiscard]] inline double asin_rt(double value, bool returnDegrees = false) {
    double result = std::asin(value);
    return returnDegrees ? radToDeg(result) : result;
}

// acos
template<typename T>
[[nodiscard]] constexpr T acos(T value, bool returnDegrees = false) {
    T result = detail::acos_float(value);
    return returnDegrees ? radToDeg(result) : result;
}
[[nodiscard]] inline float acos_rt(float value, bool returnDegrees = false) {
    float result = std::acos(value);
    return returnDegrees ? radToDeg(result) : result;
}
[[nodiscard]] inline double acos_rt(double value, bool returnDegrees = false) {
    double result = std::acos(value);
    return returnDegrees ? radToDeg(result) : result;
}

// atan
[[nodiscard]] constexpr float atan(float value, bool returnDegrees = false) {
    float result = detail::acos_float(1.0f / detail::sqrt_float(1.0f + value * value));
    if (value < 0.0f) result = -result;
    return returnDegrees ? radToDeg(result) : result;
}
[[nodiscard]] constexpr double atan(double value, bool returnDegrees = false) {
    double result = detail::acos_double(1.0 / detail::sqrt_double(1.0 + value * value));
    if (value < 0.0) result = -result;
    return returnDegrees ? radToDeg(result) : result;
}

// atan2，这个好用
[[nodiscard]] constexpr float atan2(float y, float x, bool returnDegrees = false) {
    if (x > 0.0f) return atan(y / x, returnDegrees);
    if (x < 0.0f) {
        float result = atan(y / x, false);
        result = (y >= 0.0f) ? result + PI : result - PI;
        return returnDegrees ? radToDeg(result) : result;
    }
    return (y > 0.0f) ? (returnDegrees ? 90.0f : PI_2) : ((y < 0.0f) ? (returnDegrees ? -90.0f : -PI_2) : 0.0f);
}
[[nodiscard]] constexpr double atan2(double y, double x, bool returnDegrees = false) {
    if (x > 0.0) return atan(y / x, returnDegrees);
    if (x < 0.0) {
        double result = atan(y / x, false);
        result = (y >= 0.0) ? result + PI : result - PI;
        return returnDegrees ? radToDeg(result) : result;
    }
    return (y > 0.0) ? (returnDegrees ? 90.0 : PI_2) : ((y < 0.0) ? (returnDegrees ? -90.0 : -PI_2) : 0.0);
}

// pow，幂运算，constexpr实现
constexpr float pow(float base, float exponent) {
    if (exponent == 0.0f) return 1.0f;
    if (base == 0.0f) return 0.0f;
    if (exponent == 1.0f) return base;
    
    bool negExp = false;
    float exp = exponent;
    if (exponent < 0.0f) {
        negExp = true;
        exp = -exponent;
    }
    
    int intExp = (int)exp;
    float fracExp = exp - (float)intExp;
    
    float result = 1.0f;
    float b = base;
    int e = intExp;
    while (e > 0) {
        if (e & 1) result *= b;
        b *= b;
        e >>= 1;
    }
    
    if (fracExp > 0.001f) {
        float lnBase = 0.0f;
        float x = (base - 1.0f) / (base + 1.0f);
        float x2 = x * x;
        float term = x;
        for (int i = 0; i < 10; ++i) {
            lnBase += term / (2.0f * (float)i + 1.0f);
            term *= x2;
        }
        lnBase *= 2.0f;
        
        float expVal = lnBase * fracExp;
        float eResult = 1.0f + expVal;
        float term2 = expVal;
        for (int i = 2; i <= 8; ++i) {
            term2 *= expVal / (float)i;
            eResult += term2;
        }
        
        result *= eResult;
    }
    
    return negExp ? (1.0f / result) : result;
}

constexpr double pow(double base, double exponent) {
    if (exponent == 0.0) return 1.0;
    if (base == 0.0) return 0.0;
    if (exponent == 1.0) return base;
    
    bool negExp = false;
    double exp = exponent;
    if (exponent < 0.0) {
        negExp = true;
        exp = -exponent;
    }
    
    int intExp = (int)exp;
    double fracExp = exp - (double)intExp;
    
    double result = 1.0;
    double b = base;
    int e = intExp;
    while (e > 0) {
        if (e & 1) result *= b;
        b *= b;
        e >>= 1;
    }
    
    if (fracExp > 0.001) {
        double lnBase = 0.0;
        double x = (base - 1.0) / (base + 1.0);
        double x2 = x * x;
        double term = x;
        for (int i = 0; i < 10; ++i) {
            lnBase += term / (2.0 * (double)i + 1.0);
            term *= x2;
        }
        lnBase *= 2.0;
        
        double expVal = lnBase * fracExp;
        double eResult = 1.0 + expVal;
        double term2 = expVal;
        for (int i = 2; i <= 8; ++i) {
            term2 *= expVal / (double)i;
            eResult += term2;
        }
        
        result *= eResult;
    }
    
    return negExp ? (1.0 / result) : result;
}

// cbrt，立方根，牛顿迭代法
constexpr float cbrt(float x) {
    if (x == 0.0f) return 0.0f;
    bool neg = x < 0.0f;
    float val = neg ? -x : x;
    
    float guess = val;
    for (int i = 0; i < 10; ++i) {
        guess = (2.0f * guess + val / (guess * guess)) / 3.0f;
    }
    
    return neg ? -guess : guess;
}

constexpr double cbrt(double x) {
    if (x == 0.0) return 0.0;
    bool neg = x < 0.0;
    double val = neg ? -x : x;
    
    double guess = val;
    for (int i = 0; i < 10; ++i) {
        guess = (2.0 * guess + val / (guess * guess)) / 3.0;
    }
    
    return neg ? -guess : guess;
}

// 逆线性插值，给定a, b, value，反推t
constexpr float inverseLerp(float a, float b, float value) {
    if (Math::abs(b - a) < EPSILON) return 0.0f;
    return (value - a) / (b - a);
}
constexpr double inverseLerp(double a, double b, double value) {
    if (Math::abs(b - a) < DOUBLE_EPSILON) return 0.0;
    return (value - a) / (b - a);
}

// repeat，循环重复
constexpr float repeat(float value, float length) {
    return value - floor(value / length) * length;
}
constexpr double repeat(double value, double length) {
    return value - floor(value / length) * length;
}

// pingpong，来回振荡
constexpr float pingPong(float value, float length) {
    value = repeat(value, length * 2.0f);
    return length - abs(value - length);
}
constexpr double pingPong(double value, double length) {
    value = repeat(value, length * 2.0);
    return length - abs(value - length);
}

// damp，阻尼，平滑趋向目标
float damp(float current, float target, float smoothing, float deltaTime);
double damp(double current, double target, double smoothing, double deltaTime);

// 近似相等，相对误差版本
constexpr bool approximatelyEqualRelative(float a, float b, float relativeEpsilon = 1e-5f) {
    float absA = abs(a);
    float absB = abs(b);
    float diff = abs(a - b);
    if (a == 0.0f || b == 0.0f || absA + absB < EPSILON) {
        return diff < EPSILON;
    }
    return diff / (absA + absB) < relativeEpsilon;
}
constexpr bool approximatelyEqualRelative(double a, double b, double relativeEpsilon = 1e-10) {
    double absA = abs(a);
    double absB = abs(b);
    double diff = abs(a - b);
    if (a == 0.0 || b == 0.0 || absA + absB < DOUBLE_EPSILON) {
        return diff < DOUBLE_EPSILON;
    }
    return diff / (absA + absB) < relativeEpsilon;
}

// fastInvSqrt，快速近似倒数，Quake3那个魔法数字的改进版
constexpr float fastInvSqrt(float x) {
    if (x <= 0.0f) return 0.0f;
    float guess = 1.0f / sqrt(x);
    return guess * (1.5f - 0.5f * x * guess * guess);
}
constexpr double fastInvSqrt(double x) {
    if (x <= 0.0) return 0.0;
    double guess = 1.0 / sqrt(x);
    return guess * (1.5 - 0.5 * x * guess * guess);
}

// copysign，复制符号
constexpr float copysign(float x, float y) {
    return (y >= 0.0f) ? abs(x) : -abs(x);
}
constexpr double copysign(double x, double y) {
    return (y >= 0.0) ? abs(x) : -abs(x);
}

// fmod，取模
constexpr float fmod(float x, float y) {
    if (y == 0.0f) return 0.0f;
    return x - y * static_cast<float>(static_cast<int>(x / y));
}
constexpr double fmod(double x, double y) {
    if (y == 0.0) return 0.0;
    return x - y * static_cast<double>(static_cast<int>(x / y));
}

// 阶乘
constexpr int factorial(int n) {
    return (n <= 1) ? 1 : n * factorial(n - 1);
}

// 组合数，n选k
constexpr int combinations(int n, int k) {
    if (k < 0 || k > n) return 0;
    if (k == 0 || k == n) return 1;
    if (k > n / 2) k = n - k;
    int result = 1;
    for (int i = 1; i <= k; ++i) {
        result = result * (n - i + 1) / i;
    }
    return result;
}

// 判断是不是2的幂
constexpr bool isPowerOfTwo(int x) {
    return x > 0 && (x & (x - 1)) == 0;
}

// 向上取整到2的幂，内存对齐经常用
constexpr int nextPowerOfTwo(int x) {
    if (x <= 0) return 1;
    x--;
    x |= x >> 1;
    x |= x >> 2;
    x |= x >> 4;
    x |= x >> 8;
    x |= x >> 16;
    return x + 1;
}

// 最大公约数
constexpr int gcd(int a, int b) {
    return b == 0 ? a : gcd(b, a % b);
}

// 最小公倍数
constexpr int lcm(int a, int b) {
    if (a == 0 || b == 0) return 0;
    return abs(a * b) / gcd(a, b);
}

} // namespace Math

} // namespace AstraAlgebra
