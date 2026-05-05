// NumericalMethods - 数值方法，插值、积分、求根那些，物理模拟经常用

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Core/MathUtils.h>
#include <vector>
#include <cmath>

namespace AstraAlgebra {

namespace Numerics {
    // 插值类型
    enum class InterpolationType {
        Linear,       // 线性插值
        CubicSpline,  // 三次样条
        CatmullRom,   // Catmull-Rom样条
        Bezier,       // 贝塞尔曲线
        Hermite       // 埃尔米特
    };
    
    // 积分类型
    enum class IntegrationType {
        Euler,        // 欧拉积分
        Verlet,       // Verlet积分
        RungeKutta4,  // 四阶龙格-库塔
        Midpoint      // 中点法
    };
    
    // 线性插值，最基础的
    template <typename T>
    T lerp(const T& a, const T& b, float t) {
        return a + (b - a) * t;
    }
    
    // 三次样条插值，平滑过渡
    template <typename T>
    T cubicSpline(const std::vector<T>& points, float t) {
        if (points.size() < 2) {
            return points.empty() ? T() : points[0];
        }
        
        size_t numSegments = points.size() - 1;
        float segment = t * numSegments;
        size_t idx = static_cast<size_t>(std::floor(segment));
        float u = segment - idx;
        
        idx = Math::clamp(static_cast<int>(idx), 0, static_cast<int>(numSegments - 1));
        
        const T& p0 = points[idx];
        const T& p1 = points[idx + 1];
        
        T m0 = (idx > 0) ? (p1 - points[idx - 1]) * 0.5f : (p1 - p0);
        T m1 = (idx < numSegments - 1) ? (points[idx + 2] - p0) * 0.5f : (p1 - p0);
        
        float u2 = u * u;
        float u3 = u2 * u;
        
        float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
        float h10 = u3 - 2.0f * u2 + u;
        float h01 = -2.0f * u3 + 3.0f * u2;
        float h11 = u3 - u2;
        
        return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;
    }
    
    // Catmull-Rom样条，游戏里做路径动画常用
    template <typename T>
    T catmullRom(const std::vector<T>& points, float t, float tension = 0.5f) {
        if (points.size() < 2) {
            return points.empty() ? T() : points[0];
        }
        
        size_t numSegments = points.size() - 1;
        float segment = t * numSegments;
        size_t idx = static_cast<size_t>(std::floor(segment));
        float u = segment - idx;
        
        idx = Math::clamp(static_cast<int>(idx), 0, static_cast<int>(numSegments - 1));
        
        const T& p0 = idx > 0 ? points[idx - 1] : points[idx];
        const T& p1 = points[idx];
        const T& p2 = points[idx + 1];
        const T& p3 = (idx < numSegments - 1) ? points[idx + 2] : points[idx + 1];
        
        float u2 = u * u;
        float u3 = u2 * u;
        
        T m0 = (p2 - p0) * tension;
        T m1 = (p3 - p1) * tension;
        
        float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
        float h10 = u3 - 2.0f * u2 + u;
        float h01 = -2.0f * u3 + 3.0f * u2;
        float h11 = u3 - u2;
        
        return h00 * p1 + h10 * m0 + h01 * p2 + h11 * m1;
    }
    
    // 贝塞尔曲线，做UI动画和路径很 smooth
    template <typename T>
    T bezier(const std::vector<T>& points, float t) {
        if (points.empty()) {
            return T();
        }
        
        std::vector<T> current = points;
        
        while (current.size() > 1) {
            std::vector<T> next;
            for (size_t i = 0; i < current.size() - 1; ++i) {
                next.push_back(Numerics::lerp(current[i], current[i + 1], t));
            }
            current = next;
        }
        
        return current[0];
    }
    
    // 埃尔米特插值
    template <typename T>
    T hermite(const T& p0, const T& m0, const T& p1, const T& m1, float t) {
        float u = t;
        float u2 = u * u;
        float u3 = u2 * u;
        
        float h00 = 2.0f * u3 - 3.0f * u2 + 1.0f;
        float h10 = u3 - 2.0f * u2 + u;
        float h01 = -2.0f * u3 + 3.0f * u2;
        float h11 = u3 - u2;
        
        return h00 * p0 + h10 * m0 + h01 * p1 + h11 * m1;
    }
    
    // 欧拉积分
    template <typename T, typename DerivativeFunc>
    T euler(const T& current, DerivativeFunc derivative, float dt) {
        return current + derivative(current) * dt;
    }
    
    // Verlet积分
    template <typename T, typename AccelerationFunc>
    T verlet(const T& current, const T& previous, AccelerationFunc acceleration, float dt) {
        return current * 2.0f - previous + acceleration(current) * dt * dt;
    }
    
    // 四阶龙格-库塔
    template <typename T, typename DerivativeFunc>
    T rungeKutta4(const T& current, DerivativeFunc derivative, float dt) {
        T k1 = derivative(current);
        T k2 = derivative(current + k1 * (dt * 0.5f));
        T k3 = derivative(current + k2 * (dt * 0.5f));
        T k4 = derivative(current + k3 * dt);
        
        return current + (k1 + k2 * 2.0f + k3 * 2.0f + k4) * (dt / 6.0f);
    }
    
    // 中点法
    template <typename T, typename DerivativeFunc>
    T midpoint(const T& current, DerivativeFunc derivative, float dt) {
        T k1 = derivative(current);
        T k2 = derivative(current + k1 * (dt * 0.5f));
        
        return current + k2 * dt;
    }
    
    // 数值微分（中心差分）
    template <typename Func>
    float derivative(Func func, float x, float h = 1e-5f) {
        return (func(x + h) - func(x - h)) / (2.0f * h);
    }
    
    // 数值积分（辛普森法则）
    template <typename Func>
    float integrate(Func func, float a, float b, int steps = 1000) {
        float h = (b - a) / steps;
        float sum = func(a) + func(b);
        
        for (int i = 1; i < steps; ++i) {
            float x = a + i * h;
            if (i % 2 == 0) {
                sum += 2.0f * func(x);
            } else {
                sum += 4.0f * func(x);
            }
        }
        
        return sum * h / 3.0f;
    }
    
    // 二分法求根
    template <typename Func>
    float bisection(Func func, float a, float b, float tolerance = 1e-6f, int maxIterations = 100) {
        float fa = func(a);
        float fb = func(b);
        
        if (fa * fb >= 0.0f) {
            return (fa < fb) ? a : b;
        }
        
        float c = a;
        for (int i = 0; i < maxIterations; ++i) {
            c = (a + b) * 0.5f;
            float fc = func(c);
            
            if (std::abs(fc) < tolerance || std::abs(b - a) < tolerance) {
                break;
            }
            
            if (fa * fc < 0.0f) {
                b = c;
                fb = fc;
            } else {
                a = c;
                fa = fc;
            }
        }
        
        return c;
    }
    
    // 牛顿法求根
    template <typename Func, typename DerivativeFunc>
    float newtonRaphson(Func func, DerivativeFunc derivative, float initialGuess, float tolerance = 1e-6f, int maxIterations = 100) {
        float x = initialGuess;
        
        for (int i = 0; i < maxIterations; ++i) {
            float fx = func(x);
            float dfx = derivative(x);
            
            if (std::abs(fx) < tolerance || std::abs(dfx) < Math::EPSILON) {
                break;
            }
            
            x = x - fx / dfx;
        }
        
        return x;
    }
    
    // 梯度下降
    template <typename T, typename CostFunc, typename GradientFunc>
    T gradientDescent(T initial, CostFunc cost, GradientFunc gradient, float learningRate = 0.01f, float tolerance = 1e-6f, int maxIterations = 1000) {
        T x = initial;
        float previousCost = cost(x);
        
        for (int i = 0; i < maxIterations; ++i) {
            T grad = gradient(x);
            x = x - grad * learningRate;
            
            float currentCost = cost(x);
            if (std::abs(currentCost - previousCost) < tolerance) {
                break;
            }
            
            previousCost = currentCost;
        }
        
        return x;
    }
}

} // namespace AstraAlgebra
