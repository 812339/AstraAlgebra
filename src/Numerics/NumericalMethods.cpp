// NumericalMethods.cpp - 数值方法实现
// 插值、积分、求根那些，物理模拟经常用

#include <AstraAlgebra/Numerics/NumericalMethods.h>
#include <AstraAlgebra/Core/MathUtils.h>
// 虽然linter报告未直接使用，但在模板实例化中作为类型参数需要此头文件
#include <functional>

namespace AstraAlgebra {

namespace Numerics {
    // 显式实例化常用类型
    template Vector3 lerp<Vector3>(const Vector3&, const Vector3&, float);
    template Vector4 lerp<Vector4>(const Vector4&, const Vector4&, float);
    template float lerp<float>(const float&, const float&, float);
    
    template Vector3 cubicSpline<Vector3>(const std::vector<Vector3>&, float);
    template Vector4 cubicSpline<Vector4>(const std::vector<Vector4>&, float);
    
    template Vector3 catmullRom<Vector3>(const std::vector<Vector3>&, float, float);
    template Vector4 catmullRom<Vector4>(const std::vector<Vector4>&, float, float);
    
    template Vector3 bezier<Vector3>(const std::vector<Vector3>&, float);
    template Vector4 bezier<Vector4>(const std::vector<Vector4>&, float);
    
    template Vector3 hermite<Vector3>(const Vector3&, const Vector3&, const Vector3&, const Vector3&, float);
    template Vector4 hermite<Vector4>(const Vector4&, const Vector4&, const Vector4&, const Vector4&, float);
    
    template Vector3 euler<Vector3, std::function<Vector3(const Vector3&)>>(const Vector3&, std::function<Vector3(const Vector3&)>, float);
    template Vector4 euler<Vector4, std::function<Vector4(const Vector4&)>>(const Vector4&, std::function<Vector4(const Vector4&)>, float);
    
    template Vector3 verlet<Vector3, std::function<Vector3(const Vector3&)>>(const Vector3&, const Vector3&, std::function<Vector3(const Vector3&)>, float);
    template Vector4 verlet<Vector4, std::function<Vector4(const Vector4&)>>(const Vector4&, const Vector4&, std::function<Vector4(const Vector4&)>, float);
    
    template Vector3 rungeKutta4<Vector3, std::function<Vector3(const Vector3&)>>(const Vector3&, std::function<Vector3(const Vector3&)>, float);
    template Vector4 rungeKutta4<Vector4, std::function<Vector4(const Vector4&)>>(const Vector4&, std::function<Vector4(const Vector4&)>, float);
    
    template Vector3 midpoint<Vector3, std::function<Vector3(const Vector3&)>>(const Vector3&, std::function<Vector3(const Vector3&)>, float);
    template Vector4 midpoint<Vector4, std::function<Vector4(const Vector4&)>>(const Vector4&, std::function<Vector4(const Vector4&)>, float);
    
    template float derivative<std::function<float(float)>>(std::function<float(float)>, float, float);
    template float integrate<std::function<float(float)>>(std::function<float(float)>, float, float, int);
    template float bisection<std::function<float(float)>>(std::function<float(float)>, float, float, float, int);
    template float newtonRaphson<std::function<float(float)>, std::function<float(float)>>(std::function<float(float)>, std::function<float(float)>, float, float, int);
    template Vector3 gradientDescent<Vector3, std::function<float(const Vector3&)>, std::function<Vector3(const Vector3&)>>(Vector3, std::function<float(const Vector3&)>, std::function<Vector3(const Vector3&)>, float, float, int);
    template Vector4 gradientDescent<Vector4, std::function<float(const Vector4&)>, std::function<Vector4(const Vector4&)>>(Vector4, std::function<float(const Vector4&)>, std::function<Vector4(const Vector4&)>, float, float, int);
    template float gradientDescent<float, std::function<float(float)>, std::function<float(float)>>(float, std::function<float(float)>, std::function<float(float)>, float, float, int);
}

} // namespace AstraAlgebra
