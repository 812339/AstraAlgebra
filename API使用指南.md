# AstraAlgebra API 使用指南

本文档提供 AstraAlgebra 数学库的详细 API 使用说明，包含常用功能的代码示例。

## 快速开始

### 1. 获取库

**方式一：使用预编译库**

下载分发包，包含以下内容：
```
AstraAlgebra/
├── include/
│   └── AstraAlgebra/          # 头文件
└── lib/
    └── libAstraAlgebra.a      # 静态库
```

**方式二：从源代码编译**

```bash
mkdir build && cd build
cmake .. -G Ninja -DBUILD_SHARED_LIBS=OFF
ninja
```

### 2. 在项目中引用

**GCC/MinGW 编译器：**
```bash
g++ main.cpp -I./AstraAlgebra/include -L./AstraAlgebra/lib -lAstraAlgebra -o myprogram
```

**MSVC 编译器（Visual Studio）：**
```bash
cl /EHsc main.cpp /I.\AstraAlgebra\include .\AstraAlgebra\lib\AstraAlgebra.lib
```

**CMake 项目：**
```cmake
# 添加头文件路径
target_include_directories(YourProject PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/AstraAlgebra/include)

# 链接静态库
target_link_libraries(YourProject PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/AstraAlgebra/lib/libAstraAlgebra.a)
```

### 3. 包含头文件

```cpp
#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Core/MathUtils.h>

using namespace AstraAlgebra;
```

### 4. 完整示例

```cpp
#include <iostream>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>

using namespace AstraAlgebra;

int main() {
    // 创建向量
    Vector3 position(1.0f, 2.0f, 3.0f);
    Vector3 direction(0.0f, 1.0f, 0.0f);
    
    // 向量运算
    float length = position.length();
    Vector3 normalized = position.normalized();
    float dot = position.dot(direction);
    
    // 创建变换矩阵
    Matrix4x4 transform = Matrix4x4::translation(position);
    
    // 创建四元数旋转（isDegrees=true 表示使用角度）
    Quaternion rotation = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    Vector3 rotated = rotation * Vector3::forward;
    
    std::cout << "Position: " << position << std::endl;
    std::cout << "Length: " << length << std::endl;
    std::cout << "Rotated: " << rotated << std::endl;
    
    return 0;
}
```

## 向量系统

### Vector2 / Vector3 / Vector4

向量是最基础的数学类型，用于表示位置、方向、速度等。

```cpp
Vector3 a(1.0f, 2.0f, 3.0f);
Vector3 b(4.0f, 5.0f, 6.0f);

// 加减乘除
Vector3 sum = a + b;
Vector3 diff = a - b;
Vector3 scaled = a * 2.0f;
Vector3 divided = a / 2.0f;

// 点乘、叉乘
float dot = a.dot(b);
Vector3 cross = a.cross(b);

// 长度、归一化
float len = a.length();
float lenSq = a.squaredLength();  // 更快的长度平方
Vector3 norm = a.normalized();

// 插值
Vector3 mid = Vector3::lerp(a, b, 0.5f);
Vector3 smooth = Vector3::smoothDamp(a, b, currentVelocity, smoothTime);

// 距离
float dist = a.distance(b);
float distSq = a.squaredDistance(b);  // 更快的距离平方

// 预定义常量
Vector3 zero = Vector3::zero;
Vector3 one = Vector3::one;
Vector3 forward = Vector3::forward;
Vector3 up = Vector3::up;
Vector3 right = Vector3::right;
```

**注意事项：**
- `normalized()` 返回新的归一化向量，不会修改原向量
- 除零时库内有保护机制，但建议在使用前检查
- 使用 `squaredLength()` 和 `squaredDistance()` 可以避免开方运算，性能更好

## 矩阵系统

### Matrix2x2 / Matrix3x3 / Matrix4x4

矩阵用于表示变换（平移、旋转、缩放）和投影。

```cpp
// 单位矩阵
Matrix4x4 identity = Matrix4x4::identity;

// 平移矩阵
Matrix4x4 trans = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));

// 旋转矩阵
Matrix4x4 rotX = Matrix4x4::rotationX(90.0f, true);  // true 表示使用角度
Matrix4x4 rotY = Matrix4x4::rotationY(45.0f, true);
Matrix4x4 rotZ = Matrix4x4::rotationZ(30.0f, true);

// 缩放矩阵
Matrix4x4 scale = Matrix4x4::scale(Vector3(2.0f, 2.0f, 2.0f));

// 组合变换（注意顺序：从右到左应用）
Matrix4x4 transform = trans * rotY * scale;

// 变换点
Vector3 point(1.0f, 0.0f, 0.0f);
Vector3 result = transform * point;

// 逆矩阵、转置矩阵
Matrix4x4 inv = transform.inverse();
Matrix4x4 t = transform.transposed();

// 观察矩阵（相机）
Matrix4x4 view = Matrix4x4::lookAt(
    Vector3(0.0f, 0.0f, 5.0f),  // 相机位置
    Vector3(0.0f, 0.0f, 0.0f),  // 目标点
    Vector3::up                  // 上方向
);

// 透视投影矩阵
Matrix4x4 proj = Matrix4x4::perspective(
    60.0f,    // 视野角度
    16.0f/9.0f,  // 宽高比
    0.1f,    // 近裁剪面
    100.0f   // 远裁剪面
);

// 正交投影矩阵
Matrix4x4 ortho = Matrix4x4::orthographic(-10.0f, 10.0f, -10.0f, 10.0f, 0.1f, 100.0f);
```

**注意事项：**
- 矩阵乘法不满足交换律，顺序很重要
- 变换顺序一般为：`平移 * 旋转 * 缩放`
- 使用 `inverse()` 时确保矩阵可逆

## 四元数

### Quaternion

四元数用于表示 3D 旋转，相比欧拉角可以避免万向节死锁问题。

```cpp
// 从轴角创建
Quaternion q = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);

// 从欧拉角创建
Quaternion q2 = Quaternion::fromEulerAngles(Vector3(0.0f, 90.0f, 0.0f), true);

// 旋转方向
Vector3 forward = Vector3::forward;
Vector3 rotated = q * forward;

// 插值
Quaternion slerped = q.slerp(q2, 0.5f);    // 球面线性插值（平滑）
Quaternion nlerped = q.nlerp(q2, 0.5f);    // 归一化线性插值（快速）
Quaternion fast = q.fastSlerp(q2, 0.5f);   // 快速 Slerp 近似

// 从 A 方向到 B 方向的旋转
Quaternion fromTo = Quaternion::fromToRotation(Vector3::forward, Vector3::right);

// 朝向某个方向
Quaternion look = Quaternion::lookRotation(Vector3::right, Vector3::up);

// 转换为欧拉角
Vector3 euler = q.toEulerAngles(true);

// 逆旋转
Quaternion inv = q.inverse();

// 角度差
float angle = Quaternion::angle(q, q2);
```

**注意事项：**
- 四元数乘法不满足交换律
- 建议定期归一化以避免精度损失
- `fastSlerp` 使用快速近似算法，性能更好但精度略有降低

## 几何体与碰撞检测

### 几何体类型

```cpp
// 射线
Ray ray(origin, direction);

// 球体
Sphere sphere(center, radius);

// 平面
Plane(normal, distance);

// AABB 包围盒
Bounds(min, max);

// 三角形
Triangle(v1, v2, v3);

// 胶囊体
Capsule(p1, p2, radius);

// 圆锥体
Cone(apex, axis, height, radius);

// 圆柱体
Cylinder(center, axis, height, radius);
```

### 碰撞检测

```cpp
#include <AstraAlgebra/Geometry/GJK.h>

// 射线与球体相交
float t = ray.intersect(sphere);
if (t > 0) {
    Vector3 hitPoint = ray.pointAt(t);
}

// GJK 碰撞检测（支持所有凸体组合）
bool hit = gjkIntersect(sphere, capsule);

// 获取详细碰撞信息
GJKResult result = gjkIntersectionDetails(sphere, capsule);
if (result.intersect) {
    Vector3 normal = result.normal;       // 碰撞法线
    float penetration = result.penetration;  // 穿透深度
    Vector3 contactPoint = result.contactPoint;  // 接触点
}
```

## 数值方法

### 插值

```cpp
#include <AstraAlgebra/Numerics/NumericalMethods.h>

using namespace AstraAlgebra::Numerics;

// 线性插值
float lerp = Math::lerp(0.0f, 10.0f, 0.5f);  // 5.0

// 平滑插值
float smooth = Math::smoothStep(0.0f, 1.0f, 0.5f);

// Catmull-Rom 样条插值
std::vector<Vector3> points = { ... };
Vector3 interpolated = catmullRom(points, 0.5f);

// 贝塞尔曲线
Vector3 bezier = bezierCurve(p0, p1, p2, p3, t);
```

### 数值积分

```cpp
// 四阶龙格-库塔
auto derivative = [](const Vector3& state) { return ...; };
Vector3 next = rungeKutta4(current, derivative, dt);

// 辛普森积分
float integral = simpson(func, a, b, n);
```

### 求根算法

```cpp
// 牛顿法
auto func = [](float x) { return x * x - 2.0f; };
auto deriv = [](float x) { return 2.0f * x; };
float root = newtonRaphson(func, deriv, 1.0f);

// 二分法
float root2 = bisection(func, 0.0f, 2.0f);
```

## 颜色空间

```cpp
#include <AstraAlgebra/Numerics/ColorSpace.h>

// RGB 转 HSV
ColorRGB rgb(1.0f, 0.0f, 0.0f);
ColorHSV hsv = ColorSpace::rgbToHsv(rgb);

// HSV 转 RGB
ColorRGB rgb2 = ColorSpace::hsvToRgb(hsv);

// RGB 转 HSL
ColorHSL hsl = ColorSpace::rgbToHsl(rgb);

// RGB 转 CMYK
ColorCMYK cmyk = ColorSpace::rgbToCmyk(rgb);

// 预定义颜色
ColorRGB red = ColorSpace::red();
ColorRGB blue = ColorSpace::blue();
ColorRGB green = ColorSpace::green();
```

## 随机数

```cpp
#include <AstraAlgebra/Numerics/Random.h>

// 随机整数
int val = g_Random.nextInt(0, 10);

// 随机浮点数
float f = g_Random.nextFloat(0.0f, 1.0f);

// 随机布尔
bool b = g_Random.nextBool();

// 随机向量
Vector3 v = g_Random.nextVector3(-1.0f, 1.0f);

// 随机方向
Vector3 dir = g_Random.randomDirection();

// 随机旋转
Quaternion rot = g_Random.randomRotation();

// 设置种子
g_Random.seed(12345);
```

## GPU 加速

适用于 NVIDIA 显卡，可加速批量运算。

```cpp
#include <AstraAlgebra/Core/GPU.h>

// 初始化 GPU 上下文
GPUContext::getInstance().initialize(GPUDeviceType::CUDA);

// 批量向量加法
const int count = 10000;
std::vector<Vector3> a(count), b(count), result(count);
GPU::vectorAdd(a.data(), b.data(), result.data(), count);

// 批量向量缩放
GPU::vectorScale(a.data(), 2.0f, result.data(), count);

// 批量射线与球体相交
std::vector<Ray> rays(count);
std::vector<Sphere> spheres(count);
std::vector<float> distances(count);
GPU::raySphereIntersect(rays.data(), spheres.data(), distances.data(), count);
```

## Transform 层级系统

```cpp
#include <AstraAlgebra/Core/Transform.h>

// 创建父节点
Transform parent;
parent.setPosition(Vector3(1.0f, 0.0f, 0.0f));
parent.setRotation(Quaternion::fromAxisAngle(Vector3::up, 45.0f, true));

// 创建子节点
Transform child;
child.setPosition(Vector3(2.0f, 0.0f, 0.0f));
child.setParent(&parent);

// 获取世界坐标
Vector3 worldPos = child.getWorldPosition();
Quaternion worldRot = child.getWorldRotation();
Matrix4x4 worldMatrix = child.getWorldMatrix();

// 世界坐标转局部坐标
Vector3 localPos = parent.inverseTransformPoint(worldPos);
```

## 常见问题

### 链接错误 "undefined reference"

确保正确链接了静态库：
```bash
g++ main.cpp -I./dist/include -L./dist/lib -lAstraAlgebra -o myprogram
```

### 头文件找不到

确保正确指定了头文件路径：
```bash
-I./dist/include
```

### SIMD 未生效

编译时添加优化选项：
```bash
# GCC/Clang
-mavx2 -mavx -msse2

# 或使用 CMake 自动检测
cmake .. -DENABLE_SIMD=ON
```

### Windows MSVC 链接错误

使用 `.lib` 格式：
```bash
cl /EHsc main.cpp /I.\dist\include .\dist\lib\AstraAlgebra.lib
```
