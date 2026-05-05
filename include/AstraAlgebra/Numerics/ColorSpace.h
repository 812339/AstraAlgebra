// ColorSpace - 颜色空间转换，RGB、HSV、HSL、CMYK、XYZ、Lab那些

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Core/MathConstants.h>

namespace AstraAlgebra {

// 颜色结构体
struct ColorRGB {
    float r, g, b;
    
    constexpr ColorRGB() : r(0.0f), g(0.0f), b(0.0f) {}
    constexpr ColorRGB(float r, float g, float b) : r(r), g(g), b(b) {}
    constexpr ColorRGB(float gray) : r(gray), g(gray), b(gray) {}
    
    Vector3 toVector3() const { return Vector3(r, g, b); }
    Vector4 toVector4(float a = 1.0f) const { return Vector4(r, g, b, a); }
    
    static constexpr ColorRGB fromVector3(const Vector3& v) {
        return ColorRGB(v.x, v.y, v.z);
    }
};

struct ColorRGBA {
    float r, g, b, a;
    
    constexpr ColorRGBA() : r(0.0f), g(0.0f), b(0.0f), a(1.0f) {}
    constexpr ColorRGBA(float r, float g, float b, float a = 1.0f) : r(r), g(g), b(b), a(a) {}
    
    Vector4 toVector4() const { return Vector4(r, g, b, a); }
    
    static constexpr ColorRGBA fromVector4(const Vector4& v) {
        return ColorRGBA(v.x, v.y, v.z, v.w);
    }
};

struct ColorHSV {
    float h, s, v; // h: [0, 360], s: [0, 1], v: [0, 1]
    
    constexpr ColorHSV() : h(0.0f), s(0.0f), v(0.0f) {}
    constexpr ColorHSV(float h, float s, float v) : h(h), s(s), v(v) {}
};

struct ColorHSL {
    float h, s, l; // h: [0, 360], s: [0, 1], l: [0, 1]
    
    constexpr ColorHSL() : h(0.0f), s(0.0f), l(0.0f) {}
    constexpr ColorHSL(float h, float s, float l) : h(h), s(s), l(l) {}
};

struct ColorCMYK {
    float c, m, y, k; // [0, 1]
    
    constexpr ColorCMYK() : c(0.0f), m(0.0f), y(0.0f), k(0.0f) {}
    constexpr ColorCMYK(float c, float m, float y, float k) : c(c), m(m), y(y), k(k) {}
};

struct ColorXYZ {
    float x, y, z;
    
    constexpr ColorXYZ() : x(0.0f), y(0.0f), z(0.0f) {}
    constexpr ColorXYZ(float x, float y, float z) : x(x), y(y), z(z) {}
};

struct ColorLab {
    float l, a, b; // l: [0, 100], a: [-128, 127], b: [-128, 127]
    
    constexpr ColorLab() : l(0.0f), a(0.0f), b(0.0f) {}
    constexpr ColorLab(float l, float a, float b) : l(l), a(a), b(b) {}
};

// 颜色空间转换工具类
class ColorSpace {
public:
    // RGB转HSV/HSL/CMYK/XYZ/Lab
    static constexpr ColorHSV rgbToHsv(const ColorRGB& rgb);
    static constexpr ColorHSL rgbToHsl(const ColorRGB& rgb);
    static constexpr ColorCMYK rgbToCmyk(const ColorRGB& rgb);
    static constexpr ColorXYZ rgbToXyz(const ColorRGB& rgb);
    static constexpr ColorLab rgbToLab(const ColorRGB& rgb);
    
    // 反向转换
    static constexpr ColorRGB hsvToRgb(const ColorHSV& hsv);
    static constexpr ColorRGB hslToRgb(const ColorHSL& hsl);
    static constexpr ColorRGB cmykToRgb(const ColorCMYK& cmyk);
    static constexpr ColorRGB xyzToRgb(const ColorXYZ& xyz);
    static constexpr ColorRGB labToRgb(const ColorLab& lab);
    
    // XYZ跟Lab互转
    static constexpr ColorLab xyzToLab(const ColorXYZ& xyz);
    static constexpr ColorXYZ labToXyz(const ColorLab& lab);
    
    // 颜色操作
    static constexpr ColorRGB lerp(const ColorRGB& a, const ColorRGB& b, float t);  // 插值
    static constexpr ColorRGB clamp(const ColorRGB& c);  // 限制范围
    static constexpr ColorRGB gammaCorrect(const ColorRGB& c, float gamma = 2.2f);  // Gamma校正
    static constexpr ColorRGB linearToSrgb(const ColorRGB& linear);  // 线性转sRGB
    static constexpr ColorRGB srgbToLinear(const ColorRGB& srgb);  // sRGB转线性
    
    // 颜色距离（感知均匀）
    static float colorDistance(const ColorLab& a, const ColorLab& b);
    
    // 预定义颜色
    static constexpr ColorRGB red() { return ColorRGB(1.0f, 0.0f, 0.0f); }
    static constexpr ColorRGB green() { return ColorRGB(0.0f, 1.0f, 0.0f); }
    static constexpr ColorRGB blue() { return ColorRGB(0.0f, 0.0f, 1.0f); }
    static constexpr ColorRGB white() { return ColorRGB(1.0f, 1.0f, 1.0f); }
    static constexpr ColorRGB black() { return ColorRGB(0.0f, 0.0f, 0.0f); }
    static constexpr ColorRGB yellow() { return ColorRGB(1.0f, 1.0f, 0.0f); }
    static constexpr ColorRGB cyan() { return ColorRGB(0.0f, 1.0f, 1.0f); }
    static constexpr ColorRGB magenta() { return ColorRGB(1.0f, 0.0f, 1.0f); }
    static constexpr ColorRGB orange() { return ColorRGB(1.0f, 0.5f, 0.0f); }
    static constexpr ColorRGB purple() { return ColorRGB(0.5f, 0.0f, 1.0f); }
    static constexpr ColorRGB pink() { return ColorRGB(1.0f, 0.75f, 0.8f); }
    static constexpr ColorRGB gray() { return ColorRGB(0.5f, 0.5f, 0.5f); }

private:
    static constexpr float clamp01(float v) {
        return (v < 0.0f) ? 0.0f : ((v > 1.0f) ? 1.0f : v);
    }
    
    static constexpr float hueToRgb(float p, float q, float t);
    static constexpr float labF(float t);
    static constexpr float labFInv(float t);
};

// ========== 内联实现 ==========

inline constexpr ColorHSV ColorSpace::rgbToHsv(const ColorRGB& rgb) {
    float r = clamp01(rgb.r);
    float g = clamp01(rgb.g);
    float b = clamp01(rgb.b);
    
    float max = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    float min = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    float delta = max - min;
    
    float h = 0.0f, s = 0.0f, v = max;
    
    if (delta > 1e-6f) {
        s = delta / max;
        if (max == r) {
            h = 60.0f * ((g - b) / delta);
        } else if (max == g) {
            h = 60.0f * (2.0f + (b - r) / delta);
        } else {
            h = 60.0f * (4.0f + (r - g) / delta);
        }
        if (h < 0.0f) h += 360.0f;
    }
    
    return ColorHSV(h, s, v);
}

inline constexpr ColorRGB ColorSpace::hsvToRgb(const ColorHSV& hsv) {
    float h = hsv.h;
    float s = clamp01(hsv.s);
    float v = clamp01(hsv.v);
    
    if (s < 1e-6f) {
        return ColorRGB(v, v, v);
    }
    
    h = h / 60.0f;
    int i = (int)h;
    float f = h - (float)i;
    float p = v * (1.0f - s);
    float q = v * (1.0f - s * f);
    float t = v * (1.0f - s * (1.0f - f));
    
    switch (i % 6) {
        case 0: return ColorRGB(v, t, p);
        case 1: return ColorRGB(q, v, p);
        case 2: return ColorRGB(p, v, t);
        case 3: return ColorRGB(p, q, v);
        case 4: return ColorRGB(t, p, v);
        case 5: return ColorRGB(v, p, q);
        default: return ColorRGB(0.0f, 0.0f, 0.0f);
    }
}

inline constexpr ColorHSL ColorSpace::rgbToHsl(const ColorRGB& rgb) {
    float r = clamp01(rgb.r);
    float g = clamp01(rgb.g);
    float b = clamp01(rgb.b);
    
    float max = (r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b);
    float min = (r < g) ? ((r < b) ? r : b) : ((g < b) ? g : b);
    float delta = max - min;
    
    float h = 0.0f, s = 0.0f, l = (max + min) * 0.5f;
    
    if (delta > 1e-6f) {
        s = (l > 0.5f) ? (delta / (2.0f - max - min)) : (delta / (max + min));
        
        if (max == r) {
            h = 60.0f * ((g - b) / delta);
        } else if (max == g) {
            h = 60.0f * (2.0f + (b - r) / delta);
        } else {
            h = 60.0f * (4.0f + (r - g) / delta);
        }
        if (h < 0.0f) h += 360.0f;
    }
    
    return ColorHSL(h, s, l);
}

inline constexpr ColorRGB ColorSpace::hslToRgb(const ColorHSL& hsl) {
    float h = hsl.h;
    float s = clamp01(hsl.s);
    float l = clamp01(hsl.l);
    
    if (s < 1e-6f) {
        return ColorRGB(l, l, l);
    }
    
    float q = (l < 0.5f) ? (l * (1.0f + s)) : (l + s - l * s);
    float p = 2.0f * l - q;
    
    float r = hueToRgb(p, q, h / 360.0f + 1.0f / 3.0f);
    float g = hueToRgb(p, q, h / 360.0f);
    float b = hueToRgb(p, q, h / 360.0f - 1.0f / 3.0f);
    
    return ColorRGB(r, g, b);
}

inline constexpr float ColorSpace::hueToRgb(float p, float q, float t) {
    if (t < 0.0f) t += 1.0f;
    if (t > 1.0f) t -= 1.0f;
    if (t < 1.0f / 6.0f) return p + (q - p) * 6.0f * t;
    if (t < 1.0f / 2.0f) return q;
    if (t < 2.0f / 3.0f) return p + (q - p) * (2.0f / 3.0f - t) * 6.0f;
    return p;
}

inline constexpr ColorCMYK ColorSpace::rgbToCmyk(const ColorRGB& rgb) {
    float r = clamp01(rgb.r);
    float g = clamp01(rgb.g);
    float b = clamp01(rgb.b);
    
    float k = 1.0f - ((r > g) ? ((r > b) ? r : b) : ((g > b) ? g : b));
    
    if (k > 1.0f - 1e-6f) {
        return ColorCMYK(0.0f, 0.0f, 0.0f, 1.0f);
    }
    
    float c = (1.0f - r - k) / (1.0f - k);
    float m = (1.0f - g - k) / (1.0f - k);
    float y = (1.0f - b - k) / (1.0f - k);
    
    return ColorCMYK(c, m, y, k);
}

inline constexpr ColorRGB ColorSpace::cmykToRgb(const ColorCMYK& cmyk) {
    float c = clamp01(cmyk.c);
    float m = clamp01(cmyk.m);
    float y = clamp01(cmyk.y);
    float k = clamp01(cmyk.k);
    
    float r = (1.0f - c) * (1.0f - k);
    float g = (1.0f - m) * (1.0f - k);
    float b = (1.0f - y) * (1.0f - k);
    
    return ColorRGB(r, g, b);
}

inline constexpr ColorXYZ ColorSpace::rgbToXyz(const ColorRGB& rgb) {
    float r = srgbToLinear(rgb).r;
    float g = srgbToLinear(rgb).g;
    float b = srgbToLinear(rgb).b;
    
    float x = r * 0.4124564f + g * 0.3575761f + b * 0.1804375f;
    float y = r * 0.2126729f + g * 0.7151522f + b * 0.0721750f;
    float z = r * 0.0193339f + g * 0.1191920f + b * 0.9503041f;
    
    return ColorXYZ(x, y, z);
}

inline constexpr ColorRGB ColorSpace::xyzToRgb(const ColorXYZ& xyz) {
    float r = xyz.x * 3.2404542f + xyz.y * -1.5371385f + xyz.z * -0.4985314f;
    float g = xyz.x * -0.9692660f + xyz.y * 1.8760108f + xyz.z * 0.0415560f;
    float b = xyz.x * 0.0556434f + xyz.y * -0.2040259f + xyz.z * 1.0572252f;
    
    return linearToSrgb(ColorRGB(r, g, b));
}

inline constexpr float ColorSpace::labF(float t) {
    float delta = 6.0f / 29.0f;
    float delta2 = delta * delta;
    float delta3 = delta2 * delta;
    
    return (t > delta3) ? Math::cbrt(t) : (t / (3.0f * delta2) + 4.0f / 29.0f);
}

inline constexpr float ColorSpace::labFInv(float t) {
    float delta = 6.0f / 29.0f;
    return (t > delta) ? (t * t * t) : (3.0f * delta * delta * (t - 4.0f / 29.0f));
}

inline constexpr ColorLab ColorSpace::xyzToLab(const ColorXYZ& xyz) {
    const float xn = 0.95047f;
    const float yn = 1.00000f;
    const float zn = 1.08883f;
    
    float fx = labF(xyz.x / xn);
    float fy = labF(xyz.y / yn);
    float fz = labF(xyz.z / zn);
    
    float l = 116.0f * fy - 16.0f;
    float a = 500.0f * (fx - fy);
    float b = 200.0f * (fy - fz);
    
    return ColorLab(l, a, b);
}

inline constexpr ColorXYZ ColorSpace::labToXyz(const ColorLab& lab) {
    const float xn = 0.95047f;
    const float yn = 1.00000f;
    const float zn = 1.08883f;
    
    float fy = (lab.l + 16.0f) / 116.0f;
    float fx = lab.a / 500.0f + fy;
    float fz = fy - lab.b / 200.0f;
    
    float x = xn * labFInv(fx);
    float y = yn * labFInv(fy);
    float z = zn * labFInv(fz);
    
    return ColorXYZ(x, y, z);
}

inline constexpr ColorLab ColorSpace::rgbToLab(const ColorRGB& rgb) {
    return xyzToLab(rgbToXyz(rgb));
}

inline constexpr ColorRGB ColorSpace::labToRgb(const ColorLab& lab) {
    return xyzToRgb(labToXyz(lab));
}

inline constexpr ColorRGB ColorSpace::lerp(const ColorRGB& a, const ColorRGB& b, float t) {
    t = clamp01(t);
    return ColorRGB(
        a.r + (b.r - a.r) * t,
        a.g + (b.g - a.g) * t,
        a.b + (b.b - a.b) * t
    );
}

inline constexpr ColorRGB ColorSpace::clamp(const ColorRGB& c) {
    return ColorRGB(clamp01(c.r), clamp01(c.g), clamp01(c.b));
}

inline constexpr ColorRGB ColorSpace::gammaCorrect(const ColorRGB& c, float gamma) {
    return ColorRGB(
        Math::pow(clamp01(c.r), 1.0f / gamma),
        Math::pow(clamp01(c.g), 1.0f / gamma),
        Math::pow(clamp01(c.b), 1.0f / gamma)
    );
}

inline constexpr ColorRGB ColorSpace::linearToSrgb(const ColorRGB& linear) {
    return ColorRGB(
        (linear.r <= 0.0031308f) ? (12.92f * linear.r) : (1.055f * Math::pow(linear.r, 1.0f / 2.4f) - 0.055f),
        (linear.g <= 0.0031308f) ? (12.92f * linear.g) : (1.055f * Math::pow(linear.g, 1.0f / 2.4f) - 0.055f),
        (linear.b <= 0.0031308f) ? (12.92f * linear.b) : (1.055f * Math::pow(linear.b, 1.0f / 2.4f) - 0.055f)
    );
}

inline constexpr ColorRGB ColorSpace::srgbToLinear(const ColorRGB& srgb) {
    return ColorRGB(
        (srgb.r <= 0.04045f) ? (srgb.r / 12.92f) : Math::pow((srgb.r + 0.055f) / 1.055f, 2.4f),
        (srgb.g <= 0.04045f) ? (srgb.g / 12.92f) : Math::pow((srgb.g + 0.055f) / 1.055f, 2.4f),
        (srgb.b <= 0.04045f) ? (srgb.b / 12.92f) : Math::pow((srgb.b + 0.055f) / 1.055f, 2.4f)
    );
}

inline float ColorSpace::colorDistance(const ColorLab& a, const ColorLab& b) {
    float dl = a.l - b.l;
    float da = a.a - b.a;
    float db = a.b - b.b;
    return std::sqrt(dl * dl + da * da + db * db);
}

} // namespace AstraAlgebra
