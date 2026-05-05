// MathUtils.cpp - 数学工具函数实现
// 各种常用的数学操作都在这

#include <AstraAlgebra/Core/MathUtils.h>
#include <cstdlib>
#include <ctime>
#include <cmath>
#include <algorithm>

namespace AstraAlgebra {

namespace Math {

// 初始化随机种子
static bool s_RandomSeedInitialized = false;

// 球形线性插值
float slerp(float a, float b, float t) {
    // 简化版本，完整实现需要考虑四元数
    float cosOmega = a * b;
    float omega = std::acos(cosOmega);
    float sinOmega = std::sin(omega);
    
    if (std::abs(sinOmega) < EPSILON) {
        return lerp(a, b, t);
    }
    
    float t1 = std::sin((1.0f - t) * omega) / sinOmega;
    float t2 = std::sin(t * omega) / sinOmega;
    
    return a * t1 + b * t2;
}

double slerp(double a, double b, double t) {
    double cosOmega = a * b;
    double omega = std::acos(cosOmega);
    double sinOmega = std::sin(omega);
    
    if (std::abs(sinOmega) < DOUBLE_EPSILON) {
        return lerp(a, b, t);
    }
    
    double t1 = std::sin((1.0 - t) * omega) / sinOmega;
    double t2 = std::sin(t * omega) / sinOmega;
    
    return a * t1 + b * t2;
}

// 随机数生成
float random() {
    if (!s_RandomSeedInitialized) {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        s_RandomSeedInitialized = true;
    }
    return static_cast<float>(std::rand()) / static_cast<float>(RAND_MAX);
}

double randomDouble() {
    if (!s_RandomSeedInitialized) {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        s_RandomSeedInitialized = true;
    }
    return static_cast<double>(std::rand()) / static_cast<double>(RAND_MAX);
}

float randomRange(float min, float max) {
    return min + random() * (max - min);
}

double randomRange(double min, double max) {
    return min + randomDouble() * (max - min);
}

int randomInt(int min, int max) {
    if (!s_RandomSeedInitialized) {
        std::srand(static_cast<unsigned int>(std::time(nullptr)));
        s_RandomSeedInitialized = true;
    }
    return min + std::rand() % (max - min + 1);
}

// 角度处理
float normalizeAngle(float angle, bool isDegrees) {
    float range = isDegrees ? 360.0f : TWO_PI;
    float halfRange = range * 0.5f;
    
    angle = std::fmod(angle, range);
    if (angle > halfRange) {
        angle -= range;
    } else if (angle < -halfRange) {
        angle += range;
    }
    
    return angle;
}

double normalizeAngle(double angle, bool isDegrees) {
    double range = isDegrees ? 360.0 : static_cast<double>(TWO_PI);
    double halfRange = range * 0.5;
    
    angle = std::fmod(angle, range);
    if (angle > halfRange) {
        angle -= range;
    } else if (angle < -halfRange) {
        angle += range;
    }
    
    return angle;
}

float angleDifference(float a, float b, bool isDegrees) {
    float diff = b - a;
    return normalizeAngle(diff, isDegrees);
}

double angleDifference(double a, double b, bool isDegrees) {
    double diff = b - a;
    return normalizeAngle(diff, isDegrees);
}

// 符号函数
int sign(float x) {
    if (x > 0.0f) return 1;
    if (x < 0.0f) return -1;
    return 0;
}

int sign(double x) {
    if (x > 0.0) return 1;
    if (x < 0.0) return -1;
    return 0;
}

// 指数函数
float exp(float x) {
    return std::exp(x);
}

double exp(double x) {
    return std::exp(x);
}

// 对数函数
float log(float x) {
    return std::log(x);
}

double log(double x) {
    return std::log(x);
}

// 双曲函数
float sinh(float x) {
    return std::sinh(x);
}

double sinh(double x) {
    return std::sinh(x);
}

float cosh(float x) {
    return std::cosh(x);
}

double cosh(double x) {
    return std::cosh(x);
}

float tanh(float x) {
    return std::tanh(x);
}

double tanh(double x) {
    return std::tanh(x);
}

// 反双曲函数
float asinh(float x) {
    return std::asinh(x);
}

double asinh(double x) {
    return std::asinh(x);
}

float acosh(float x) {
    return std::acosh(x);
}

double acosh(double x) {
    return std::acosh(x);
}

float atanh(float x) {
    return std::atanh(x);
}

double atanh(double x) {
    return std::atanh(x);
}

// 高斯分布随机数（使用Box-Muller变换）
float randomGaussian(float mean, float stdDev) {
    static bool hasSpare = false;
    static float z1;
    
    if (hasSpare) {
        hasSpare = false;
        return mean + z1 * stdDev;
    }
    
    hasSpare = true;
    float u1, u2;
    do {
        u1 = random();
        u2 = random();
    } while (u1 <= Math::EPSILON);
    
    float z0 = std::sqrt(-2.0f * std::log(u1)) * std::cos(2.0f * Math::PI * u2);
    z1 = std::sqrt(-2.0f * std::log(u1)) * std::sin(2.0f * Math::PI * u2);
    
    return mean + z0 * stdDev;
}

double randomGaussian(double mean, double stdDev) {
    static bool hasSpare = false;
    static double z1;
    
    if (hasSpare) {
        hasSpare = false;
        return mean + z1 * stdDev;
    }
    
    hasSpare = true;
    double u1, u2;
    do {
        u1 = randomDouble();
        u2 = randomDouble();
    } while (u1 <= Math::DOUBLE_EPSILON);
    
    double z0 = std::sqrt(-2.0 * std::log(u1)) * std::cos(2.0 * Math::PI * u2);
    z1 = std::sqrt(-2.0 * std::log(u1)) * std::sin(2.0 * Math::PI * u2);
    
    return mean + z0 * stdDev;
}

// 颜色转换
// RGB到HSV转换
void rgbToHsv(float r, float g, float b, float& h, float& s, float& v) {
    float maxVal = std::max(r, std::max(g, b));
    float minVal = std::min(r, std::min(g, b));
    float delta = maxVal - minVal;
    
    // 计算V
    v = maxVal;
    
    // 计算S
    if (maxVal > 0.0f) {
        s = delta / maxVal;
    } else {
        s = 0.0f;
        h = 0.0f;
        return;
    }
    
    // 计算H
    if (delta == 0.0f) {
        h = 0.0f;
    } else if (maxVal == r) {
        h = 60.0f * ((g - b) / delta);
    } else if (maxVal == g) {
        h = 60.0f * (2.0f + (b - r) / delta);
    } else {
        h = 60.0f * (4.0f + (r - g) / delta);
    }
    
    if (h < 0.0f) {
        h += 360.0f;
    }
}

void rgbToHsv(double r, double g, double b, double& h, double& s, double& v) {
    double maxVal = std::max(r, std::max(g, b));
    double minVal = std::min(r, std::min(g, b));
    double delta = maxVal - minVal;
    
    // 计算V
    v = maxVal;
    
    // 计算S
    if (maxVal > 0.0) {
        s = delta / maxVal;
    } else {
        s = 0.0;
        h = 0.0;
        return;
    }
    
    // 计算H
    if (delta == 0.0) {
        h = 0.0;
    } else if (maxVal == r) {
        h = 60.0 * ((g - b) / delta);
    } else if (maxVal == g) {
        h = 60.0 * (2.0 + (b - r) / delta);
    } else {
        h = 60.0 * (4.0 + (r - g) / delta);
    }
    
    if (h < 0.0) {
        h += 360.0;
    }
}

// HSV到RGB转换
void hsvToRgb(float h, float s, float v, float& r, float& g, float& b) {
    if (s == 0.0f) {
        r = g = b = v;
        return;
    }
    
    float hh = h / 60.0f;
    int i = static_cast<int>(std::floor(hh));
    float ff = hh - i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - (s * ff));
    float t = v * (1.0f - (s * (1.0f - ff)));
    
    switch (i) {
        case 0:
            r = v; g = t; b = p;
            break;
        case 1:
            r = q; g = v; b = p;
            break;
        case 2:
            r = p; g = v; b = t;
            break;
        case 3:
            r = p; g = q; b = v;
            break;
        case 4:
            r = t; g = p; b = v;
            break;
        default:
            r = v; g = p; b = q;
            break;
    }
}

void hsvToRgb(double h, double s, double v, double& r, double& g, double& b) {
    if (s == 0.0) {
        r = g = b = v;
        return;
    }
    
    double hh = h / 60.0;
    int i = static_cast<int>(std::floor(hh));
    double ff = hh - i;
    double p = v * (1.0 - s);
    double q = v * (1.0 - (s * ff));
    double t = v * (1.0 - (s * (1.0 - ff)));
    
    switch (i) {
        case 0:
            r = v; g = t; b = p;
            break;
        case 1:
            r = q; g = v; b = p;
            break;
        case 2:
            r = p; g = v; b = t;
            break;
        case 3:
            r = p; g = q; b = v;
            break;
        case 4:
            r = t; g = p; b = v;
            break;
        default:
            r = v; g = p; b = q;
            break;
    }
}

// RGB到HSL转换
void rgbToHsl(float r, float g, float b, float& h, float& s, float& l) {
    float maxVal = std::max({r, g, b});
    float minVal = std::min({r, g, b});
    float delta = maxVal - minVal;
    
    // 计算L
    l = (maxVal + minVal) / 2.0f;
    
    // 计算S
    if (delta == 0.0f) {
        s = 0.0f;
        h = 0.0f;
        return;
    }
    
    if (l > 0.5f) {
        s = delta / (2.0f - maxVal - minVal);
    } else {
        s = delta / (maxVal + minVal);
    }
    
    // 计算H
    if (maxVal == r) {
        h = 60.0f * ((g - b) / delta);
    } else if (maxVal == g) {
        h = 60.0f * (2.0f + (b - r) / delta);
    } else {
        h = 60.0f * (4.0f + (r - g) / delta);
    }
    
    if (h < 0.0f) {
        h += 360.0f;
    }
}

void rgbToHsl(double r, double g, double b, double& h, double& s, double& l) {
    double maxVal = std::max({r, g, b});
    double minVal = std::min({r, g, b});
    double delta = maxVal - minVal;
    
    // 计算L
    l = (maxVal + minVal) / 2.0;
    
    // 计算S
    if (delta == 0.0) {
        s = 0.0;
        h = 0.0;
        return;
    }
    
    if (l > 0.5) {
        s = delta / (2.0 - maxVal - minVal);
    } else {
        s = delta / (maxVal + minVal);
    }
    
    // 计算H
    if (maxVal == r) {
        h = 60.0 * ((g - b) / delta);
    } else if (maxVal == g) {
        h = 60.0 * (2.0 + (b - r) / delta);
    } else {
        h = 60.0 * (4.0 + (r - g) / delta);
    }
    
    if (h < 0.0) {
        h += 360.0;
    }
}

// HSL到RGB转换
void hslToRgb(float h, float s, float l, float& r, float& g, float& b) {
    auto hue2rgb = [](float p, float q, float t) -> float {
        if (t < 0.0f) t += 1.0f;
        if (t > 1.0f) t -= 1.0f;
        if (t < 1.0f/6.0f) return p + (q - p) * 6.0f * t;
        if (t < 1.0f/2.0f) return q;
        if (t < 2.0f/3.0f) return p + (q - p) * (2.0f/3.0f - t) * 6.0f;
        return p;
    };
    
    if (s == 0.0f) {
        r = g = b = l;
    } else {
        float q = (l < 0.5f) ? (l * (1.0f + s)) : (l + s - l * s);
        float p = 2.0f * l - q;
        
        r = hue2rgb(p, q, h / 360.0f + 1.0f/3.0f);
        g = hue2rgb(p, q, h / 360.0f);
        b = hue2rgb(p, q, h / 360.0f - 1.0f/3.0f);
    }
}

void hslToRgb(double h, double s, double l, double& r, double& g, double& b) {
    auto hue2rgb = [](double p, double q, double t) -> double {
        if (t < 0.0) t += 1.0;
        if (t > 1.0) t -= 1.0;
        if (t < 1.0/6.0) return p + (q - p) * 6.0 * t;
        if (t < 1.0/2.0) return q;
        if (t < 2.0/3.0) return p + (q - p) * (2.0/3.0 - t) * 6.0;
        return p;
    };
    
    if (s == 0.0) {
        r = g = b = l;
    } else {
        double q = (l < 0.5) ? (l * (1.0 + s)) : (l + s - l * s);
        double p = 2.0 * l - q;
        
        r = hue2rgb(p, q, h / 360.0 + 1.0/3.0);
        g = hue2rgb(p, q, h / 360.0);
        b = hue2rgb(p, q, h / 360.0 - 1.0/3.0);
    }
}

// 四元数相关的辅助函数
float quatDot(float x0, float y0, float z0, float w0, float x1, float y1, float z1, float w1) {
    return x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1;
}

double quatDot(double x0, double y0, double z0, double w0, double x1, double y1, double z1, double w1) {
    return x0 * x1 + y0 * y1 + z0 * z1 + w0 * w1;
}

// 插值增强
float hermite(float v0, float v1, float m0, float m1, float t) {
    float t2 = t * t;
    float t3 = t2 * t;
    
    float h00 = 2.0f * t3 - 3.0f * t2 + 1.0f;
    float h10 = t3 - 2.0f * t2 + t;
    float h01 = -2.0f * t3 + 3.0f * t2;
    float h11 = t3 - t2;
    
    return h00 * v0 + h10 * m0 + h01 * v1 + h11 * m1;
}

double hermite(double v0, double v1, double m0, double m1, double t) {
    double t2 = t * t;
    double t3 = t2 * t;
    
    double h00 = 2.0 * t3 - 3.0 * t2 + 1.0;
    double h10 = t3 - 2.0 * t2 + t;
    double h01 = -2.0 * t3 + 3.0 * t2;
    double h11 = t3 - t2;
    
    return h00 * v0 + h10 * m0 + h01 * v1 + h11 * m1;
}

float bezier(float p0, float p1, float p2, float p3, float t) {
    float u = 1.0f - t;
    float tt = t * t;
    float uu = u * u;
    float uuu = uu * u;
    float ttt = tt * t;
    
    float p = uuu * p0;
    p += 3.0f * uu * t * p1;
    p += 3.0f * u * tt * p2;
    p += ttt * p3;
    
    return p;
}

double bezier(double p0, double p1, double p2, double p3, double t) {
    double u = 1.0 - t;
    double tt = t * t;
    double uu = u * u;
    double uuu = uu * u;
    double ttt = tt * t;
    
    double p = uuu * p0;
    p += 3.0 * uu * t * p1;
    p += 3.0 * u * tt * p2;
    p += ttt * p3;
    
    return p;
}

// 向量和矩阵辅助函数
float determinant2x2(float a, float b, float c, float d) {
    return a * d - b * c;
}

double determinant2x2(double a, double b, double c, double d) {
    return a * d - b * c;
}

float determinant3x3(float a11, float a12, float a13, float a21, float a22, float a23, float a31, float a32, float a33) {
    return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31);
}

double determinant3x3(double a11, double a12, double a13, double a21, double a22, double a23, double a31, double a32, double a33) {
    return a11 * (a22 * a33 - a23 * a32) - a12 * (a21 * a33 - a23 * a31) + a13 * (a21 * a32 - a22 * a31);
}

// 阻尼函数实现
float damp(float current, float target, float smoothing, float deltaTime) {
    float t = 1.0f - std::exp(-smoothing * deltaTime);
    return lerp(current, target, t);
}

double damp(double current, double target, double smoothing, double deltaTime) {
    double t = 1.0 - std::exp(-smoothing * deltaTime);
    return lerp(current, target, t);
}

} // namespace Math

} // namespace AstraAlgebra
