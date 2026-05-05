# AstraAlgebra - 高性能游戏引擎数学库

AstraAlgebra 是一个 C++ 数学库，主要面向游戏引擎和图形应用。除了基础的向量、矩阵、四元数运算，还集成了碰撞检测（GJK/EPA）、GPU 加速、数值方法、噪声生成、颜色空间转换等功能。

## 功能概览

| 模块 | 功能 |
|------|------|
| **向量系统** | Vector2 / Vector3 / Vector4 / Vector3d（双精度） |
| **矩阵系统** | Matrix2x2 / Matrix3x3 / Matrix4x4 / Matrix4x4d（双精度） |
| **四元数** | 旋转表示、Slerp/Nlerp/FastSlerp、fromToRotation、lookRotation |
| **几何体** | Ray / Sphere / Plane / Bounds / Triangle / Capsule / Cone / Cylinder |
| **碰撞检测** | GJK + EPA 算法，支持 7 种几何体任意组合 |
| **数值方法** | 5种插值、4种数值积分、求根算法、梯度下降 |
| **矩阵分解** | LU / QR / Cholesky / SVD / 特征值分解 / 极分解 |
| **GPU 加速** | CUDA 批量向量/矩阵运算、批量射线相交检测 |
| **SIMD 优化** | SSE2 / AVX / AVX2 自动检测与优化 |
| **工具类** | 颜色空间（RGB/HSV/HSL/CMYK）、Perlin 噪声、随机数（多种分布） |
| **Transform** | 层级变换系统，支持父子关系、世界/局部坐标转换 |
| **constexpr** | 核心类型支持编译期计算，运行时零开销 |

## 快速开始

### 环境要求

- C++17 或更高版本
- CMake 3.10 或更高版本
- 编译器：MSVC / GCC / Clang

### 构建

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -G Ninja
cmake --build . --config Release
```

**CMake 选项：**
- `ENABLE_SIMD=ON` — 启用 SIMD 优化（默认开启，自动检测 AVX2/AVX/SSE2）
- `BUILD_TESTS=ON` — 构建单元测试
- `BUILD_EXAMPLES=ON` — 构建示例程序

### 运行测试

```bash
./AstraAlgebraTests
# 232 tests, 100% pass rate
```

### 安装

```bash
cmake --install . --prefix /your/install/path
```

## 使用示例

### 向量运算

```cpp
#include <AstraAlgebra/Vector/Vector3.h>
#include <iostream>

using namespace AstraAlgebra;

int main() {
    Vector3 a(1.0f, 2.0f, 3.0f);
    Vector3 b(4.0f, 5.0f, 6.0f);

    Vector3 sum = a + b;
    float dot = a.dot(b);
    Vector3 cross = a.cross(b);
    Vector3 norm = a.normalized();

    std::cout << "Sum: " << sum << std::endl;
    std::cout << "Dot: " << dot << std::endl;
    return 0;
}
```

### 四元数旋转

```cpp
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>

using namespace AstraAlgebra;

int main() {
    // 绕 Y 轴旋转 90 度
    Quaternion rot = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);

    // 旋转向量
    Vector3 forward = Vector3::forward;
    Vector3 rotated = rot * forward;

    // 插值
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, 180.0f, true);
    Quaternion slerped = q1.slerp(q2, 0.5f);

    return 0;
}
```

### 矩阵变换

```cpp
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>

using namespace AstraAlgebra;

int main() {
    Matrix4x4 translation = Matrix4x4::translation(Vector3(5.0f, 0.0f, 0.0f));
    Matrix4x4 rotation = Matrix4x4::rotationY(90.0f, true);
    Matrix4x4 scale = Matrix4x4::scale(Vector3(2.0f, 2.0f, 2.0f));

    // 组合变换
    Matrix4x4 transform = translation * rotation * scale;

    // 变换点
    Vector3 point(1.0f, 0.0f, 0.0f);
    Vector3 result = transform * point;

    return 0;
}
```

### 碰撞检测（GJK）

```cpp
#include <AstraAlgebra/Geometry/GJK.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Capsule.h>

using namespace AstraAlgebra;

int main() {
    Sphere sphere(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    Capsule capsule(Vector3(-2.0f, 0.0f, 0.0f), Vector3(2.0f, 0.0f, 0.0f), 0.5f);

    // 快速检测
    bool hit = gjkIntersect(sphere, capsule);

    // 获取详细碰撞信息
    GJKResult result = gjkIntersectionDetails(sphere, capsule);
    if (result.intersect) {
        // result.normal      - 碰撞法线
        // result.penetration - 穿透深度
        // result.contactPoint - 接触点
    }

    return 0;
}
```

### Transform 层级系统

```cpp
#include <AstraAlgebra/Core/Transform.h>

using namespace AstraAlgebra;

int main() {
    Transform parent;
    parent.setPosition(Vector3(1.0f, 0.0f, 0.0f));

    Transform child;
    child.setPosition(Vector3(2.0f, 0.0f, 0.0f));
    child.setParent(&parent);

    // 获取世界坐标
    Vector3 worldPos = child.getWorldPosition();
    Matrix4x4 worldMatrix = child.getWorldMatrix();

    return 0;
}
```

### 数值方法

```cpp
#include <AstraAlgebra/Numerics/NumericalMethods.h>

using namespace AstraAlgebra;
using namespace AstraAlgebra::Numerics;

int main() {
    // 插值
    std::vector<Vector3> points = { ... };
    Vector3 interpolated = catmullRom(points, 0.5f);

    // 数值积分（四阶龙格-库塔）
    auto derivative = [](const Vector3& state) { return ...; };
    Vector3 next = rungeKutta4(current, derivative, dt);

    // 求根（牛顿法）
    auto func = [](float x) { return x * x - 2.0f; };
    auto deriv = [](float x) { return 2.0f * x; };
    float root = newtonRaphson(func, deriv, 1.0f);

    return 0;
}
```

### GPU 批量运算

```cpp
#include <AstraAlgebra/Core/GPU.h>

using namespace AstraAlgebra;

int main() {
    const int count = 10000;
    std::vector<Vector3> a(count), b(count), result(count);

    // GPU 加速批量运算
    GPU::vectorAdd(a.data(), b.data(), result.data(), count);
    GPU::vectorScale(a.data(), 2.0f, result.data(), count);

    // 批量射线相交
    std::vector<Ray> rays(count);
    std::vector<Sphere> spheres(count);
    std::vector<float> distances(count);
    GPU::raySphereIntersect(rays.data(), spheres.data(), distances.data(), count);

    return 0;
}
```

## 在 CMake 项目中使用

### 方式一：子项目

```cmake
add_subdirectory(AstraAlgebra)
target_link_libraries(YourProject PRIVATE AstraAlgebra)
```

### 方式二：已安装的库

```cmake
find_package(AstraAlgebra REQUIRED)
target_link_libraries(YourProject PRIVATE AstraAlgebra::AstraAlgebra)
```

### 方式三：手动链接

```bash
g++ main.cpp -I./dist/include -L./dist/lib -lAstraAlgebra -o myprogram
```

## 项目结构

```
AstraAlgebra/
├── include/AstraAlgebra/     # 头文件（公共 API）
│   ├── Core/                 # 核心工具（GPU、SIMD、序列化、Transform）
│   ├── Geometry/             # 几何体与碰撞检测（GJK）
│   ├── Matrix/               # 矩阵（2x2 ~ 4x4，含双精度）
│   ├── Numerics/             # 数值方法（插值、积分、颜色、随机数）
│   ├── Quaternion/           # 四元数
│   ├── Vector/               # 向量（2D/3D/4D，含双精度）
│   └── AstraAlgebra.h        # 统一头文件
├── src/                      # 源代码实现
├── examples/                 # 示例程序
├── test/                     # 单元测试与基准测试
└── CMakeLists.txt            # 构建配置
```

## 性能

| 优化级别 | 说明 |
|----------|------|
| **算法优化** | 四元数旋转向量（比传统方法快 90%）、代数公式矩阵求逆（快 78%） |
| **SIMD** | 自动检测 CPU 指令集，启用 AVX2/AVX/SSE2 优化 |
| **GPU** | CUDA 批量运算，适合万级以上的数据并行处理 |
| **constexpr** | 核心运算可在编译期完成，运行时零开销 |

## 文档

- [API 使用指南](API使用指南.md) — 详细的 API 调用说明
- [项目使用指南](项目使用指南.md) — 如何在你的项目中使用 AstraAlgebra

## 许可证

MIT License

---

**AstraAlgebra** — 为游戏引擎设计的高性能数学库
