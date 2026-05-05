# AstraAlgebra API Usage Guide

This document provides detailed API usage instructions for the AstraAlgebra math library, including code examples for common functionality.

## Quick Start

### 1. Getting the Library

**Option 1: Using Precompiled Library**

Download the distribution package, which includes:
```
AstraAlgebra/
├── include/
│   └── AstraAlgebra/          # Header files
└── lib/
    └── libAstraAlgebra.a      # Static library
```

**Option 2: Building from Source**

```bash
mkdir build && cd build
cmake .. -G Ninja -DBUILD_SHARED_LIBS=OFF
ninja
```

### 2. Referencing in Your Project

**GCC/MinGW Compiler:**
```bash
g++ main.cpp -I./AstraAlgebra/include -L./AstraAlgebra/lib -lAstraAlgebra -o myprogram
```

**MSVC Compiler (Visual Studio):**
```bash
cl /EHsc main.cpp /I.\AstraAlgebra\include .\AstraAlgebra\lib\AstraAlgebra.lib
```

**CMake Project:**
```cmake
# Add header file path
target_include_directories(YourProject PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/AstraAlgebra/include)

# Link static library
target_link_libraries(YourProject PRIVATE ${CMAKE_CURRENT_SOURCE_DIR}/AstraAlgebra/lib/libAstraAlgebra.a)
```

### 3. Including Headers

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

### 4. Complete Example

```cpp
#include <iostream>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>

using namespace AstraAlgebra;

int main() {
    // Create vectors
    Vector3 position(1.0f, 2.0f, 3.0f);
    Vector3 direction(0.0f, 1.0f, 0.0f);
    
    // Vector operations
    float length = position.length();
    Vector3 normalized = position.normalized();
    float dot = position.dot(direction);
    
    // Create transformation matrix
    Matrix4x4 transform = Matrix4x4::translation(position);
    
    // Create quaternion rotation (isDegrees=true means using degrees)
    Quaternion rotation = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    Vector3 rotated = rotation * Vector3::forward;
    
    std::cout << "Position: " << position << std::endl;
    std::cout << "Length: " << length << std::endl;
    std::cout << "Rotated: " << rotated << std::endl;
    
    return 0;
}
```

## Vector System

### Vector2 / Vector3 / Vector4

Vectors are the most basic math types, used for representing position, direction, velocity, etc.

```cpp
Vector3 a(1.0f, 2.0f, 3.0f);
Vector3 b(4.0f, 5.0f, 6.0f);

// Addition, subtraction, multiplication, division
Vector3 sum = a + b;
Vector3 diff = a - b;
Vector3 scaled = a * 2.0f;
Vector3 divided = a / 2.0f;

// Dot product, cross product
float dot = a.dot(b);
Vector3 cross = a.cross(b);

// Length, normalization
float len = a.length();
float lenSq = a.squaredLength();  // Faster squared length
Vector3 norm = a.normalized();

// Interpolation
Vector3 mid = Vector3::lerp(a, b, 0.5f);
Vector3 smooth = Vector3::smoothDamp(a, b, currentVelocity, smoothTime);

// Distance
float dist = a.distance(b);
float distSq = a.squaredDistance(b);  // Faster squared distance

// Predefined constants
Vector3 zero = Vector3::zero;
Vector3 one = Vector3::one;
Vector3 forward = Vector3::forward;
Vector3 up = Vector3::up;
Vector3 right = Vector3::right;
```

**Notes:**
- `normalized()` returns a new normalized vector, does not modify the original
- Division by zero is protected by the library, but it's recommended to check before use
- Use `squaredLength()` and `squaredDistance()` to avoid square root operations for better performance

## Matrix System

### Matrix2x2 / Matrix3x3 / Matrix4x4

Matrices are used for transformations (translation, rotation, scaling) and projections.

```cpp
// Identity matrix
Matrix4x4 identity = Matrix4x4::identity;

// Translation matrix
Matrix4x4 trans = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));

// Rotation matrices
Matrix4x4 rotX = Matrix4x4::rotationX(90.0f, true);  // true means using degrees
Matrix4x4 rotY = Matrix4x4::rotationY(45.0f, true);
Matrix4x4 rotZ = Matrix4x4::rotationZ(30.0f, true);

// Scaling matrix
Matrix4x4 scale = Matrix4x4::scale(Vector3(2.0f, 2.0f, 2.0f));

// Combine transformations (note the order: applied right to left)
Matrix4x4 transform = trans * rotY * scale;

// Transform a point
Vector3 point(1.0f, 0.0f, 0.0f);
Vector3 result = transform * point;

// Inverse and transpose matrices
Matrix4x4 inv = transform.inverse();
Matrix4x4 t = transform.transposed();

// View matrix (camera)
Matrix4x4 view = Matrix4x4::lookAt(
    Vector3(0.0f, 0.0f, 5.0f),  // Camera position
    Vector3(0.0f, 0.0f, 0.0f),  // Target point
    Vector3::up                  // Up direction
);

// Perspective projection matrix
Matrix4x4 proj = Matrix4x4::perspective(
    60.0f,    // Field of view angle
    16.0f/9.0f,  // Aspect ratio
    0.1f,    // Near clipping plane
    100.0f   // Far clipping plane
);

// Orthographic projection matrix
Matrix4x4 ortho = Matrix4x4::orthographic(-10.0f, 10.0f, -10.0f, 10.0f, 0.1f, 100.0f);
```

**Notes:**
- Matrix multiplication is not commutative, order matters
- Transformation order is typically: `translation * rotation * scaling`
- Ensure the matrix is invertible when using `inverse()`

## Quaternions

### Quaternion

Quaternions are used for representing 3D rotations, avoiding gimbal lock compared to Euler angles.

```cpp
// Create from axis-angle
Quaternion q = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);

// Create from Euler angles
Quaternion q2 = Quaternion::fromEulerAngles(Vector3(0.0f, 90.0f, 0.0f), true);

// Rotate a direction
Vector3 forward = Vector3::forward;
Vector3 rotated = q * forward;

// Interpolation
Quaternion slerped = q.slerp(q2, 0.5f);    // Spherical linear interpolation (smooth)
Quaternion nlerped = q.nlerp(q2, 0.5f);    // Normalized linear interpolation (fast)
Quaternion fast = q.fastSlerp(q2, 0.5f);   // Fast Slerp approximation

// Rotation from direction A to B
Quaternion fromTo = Quaternion::fromToRotation(Vector3::forward, Vector3::right);

// Look at a direction
Quaternion look = Quaternion::lookRotation(Vector3::right, Vector3::up);

// Convert to Euler angles
Vector3 euler = q.toEulerAngles(true);

// Inverse rotation
Quaternion inv = q.inverse();

// Angle difference
float angle = Quaternion::angle(q, q2);
```

**Notes:**
- Quaternion multiplication is not commutative
- Regular normalization is recommended to avoid precision loss
- `fastSlerp` uses a fast approximation algorithm, better performance but slightly lower precision

## Geometry and Collision Detection

### Geometry Types

```cpp
// Ray
Ray ray(origin, direction);

// Sphere
Sphere sphere(center, radius);

// Plane
Plane(normal, distance);

// AABB (Axis-Aligned Bounding Box)
Bounds(min, max);

// Triangle
Triangle(v1, v2, v3);

// Capsule
Capsule(p1, p2, radius);

// Cone
Cone(apex, axis, height, radius);

// Cylinder
Cylinder(center, axis, height, radius);
```

### Collision Detection

```cpp
#include <AstraAlgebra/Geometry/GJK.h>

// Ray-sphere intersection
float t = ray.intersect(sphere);
if (t > 0) {
    Vector3 hitPoint = ray.pointAt(t);
}

// GJK collision detection (supports all convex shape combinations)
bool hit = gjkIntersect(sphere, capsule);

// Get detailed collision information
GJKResult result = gjkIntersectionDetails(sphere, capsule);
if (result.intersect) {
    Vector3 normal = result.normal;       // Collision normal
    float penetration = result.penetration;  // Penetration depth
    Vector3 contactPoint = result.contactPoint;  // Contact point
}
```

## Numerical Methods

### Interpolation

```cpp
#include <AstraAlgebra/Numerics/NumericalMethods.h>

using namespace AstraAlgebra::Numerics;

// Linear interpolation
float lerp = Math::lerp(0.0f, 10.0f, 0.5f);  // 5.0

// Smooth interpolation
float smooth = Math::smoothStep(0.0f, 1.0f, 0.5f);

// Catmull-Rom spline interpolation
std::vector<Vector3> points = { ... };
Vector3 interpolated = catmullRom(points, 0.5f);

// Bezier curve
Vector3 bezier = bezierCurve(p0, p1, p2, p3, t);
```

### Numerical Integration

```cpp
// 4th-order Runge-Kutta
auto derivative = [](const Vector3& state) { return ...; };
Vector3 next = rungeKutta4(current, derivative, dt);

// Simpson's integration
float integral = simpson(func, a, b, n);
```

### Root Finding

```cpp
// Newton's method
auto func = [](float x) { return x * x - 2.0f; };
auto deriv = [](float x) { return 2.0f * x; };
float root = newtonRaphson(func, deriv, 1.0f);

// Bisection method
float root2 = bisection(func, 0.0f, 2.0f);
```

## Color Space

```cpp
#include <AstraAlgebra/Numerics/ColorSpace.h>

// RGB to HSV
ColorRGB rgb(1.0f, 0.0f, 0.0f);
ColorHSV hsv = ColorSpace::rgbToHsv(rgb);

// HSV to RGB
ColorRGB rgb2 = ColorSpace::hsvToRgb(hsv);

// RGB to HSL
ColorHSL hsl = ColorSpace::rgbToHsl(rgb);

// RGB to CMYK
ColorCMYK cmyk = ColorSpace::rgbToCmyk(rgb);

// Predefined colors
ColorRGB red = ColorSpace::red();
ColorRGB blue = ColorSpace::blue();
ColorRGB green = ColorSpace::green();
```

## Random Numbers

```cpp
#include <AstraAlgebra/Numerics/Random.h>

// Random integer
int val = g_Random.nextInt(0, 10);

// Random float
float f = g_Random.nextFloat(0.0f, 1.0f);

// Random boolean
bool b = g_Random.nextBool();

// Random vector
Vector3 v = g_Random.nextVector3(-1.0f, 1.0f);

// Random direction
Vector3 dir = g_Random.randomDirection();

// Random rotation
Quaternion rot = g_Random.randomRotation();

// Set seed
g_Random.seed(12345);
```

## GPU Acceleration

For NVIDIA GPUs, batch operations can be accelerated.

```cpp
#include <AstraAlgebra/Core/GPU.h>

// Initialize GPU context
GPUContext::getInstance().initialize(GPUDeviceType::CUDA);

// Batch vector addition
const int count = 10000;
std::vector<Vector3> a(count), b(count), result(count);
GPU::vectorAdd(a.data(), b.data(), result.data(), count);

// Batch vector scaling
GPU::vectorScale(a.data(), 2.0f, result.data(), count);

// Batch ray-sphere intersection
std::vector<Ray> rays(count);
std::vector<Sphere> spheres(count);
std::vector<float> distances(count);
GPU::raySphereIntersect(rays.data(), spheres.data(), distances.data(), count);
```

## Transform Hierarchy

```cpp
#include <AstraAlgebra/Core/Transform.h>

// Create parent node
Transform parent;
parent.setPosition(Vector3(1.0f, 0.0f, 0.0f));
parent.setRotation(Quaternion::fromAxisAngle(Vector3::up, 45.0f, true));

// Create child node
Transform child;
child.setPosition(Vector3(2.0f, 0.0f, 0.0f));
child.setParent(&parent);

// Get world coordinates
Vector3 worldPos = child.getWorldPosition();
Quaternion worldRot = child.getWorldRotation();
Matrix4x4 worldMatrix = child.getWorldMatrix();

// World to local coordinates
Vector3 localPos = parent.inverseTransformPoint(worldPos);
```

## Common Issues

### Link Error "undefined reference"

Ensure the static library is properly linked:
```bash
g++ main.cpp -I./dist/include -L./dist/lib -lAstraAlgebra -o myprogram
```

### Header File Not Found

Ensure the header file path is correctly specified:
```bash
-I./dist/include
```

### SIMD Not Enabled

Add optimization flags during compilation:
```bash
# GCC/Clang
-mavx2 -mavx -msse2

# Or use CMake auto-detection
cmake .. -DENABLE_SIMD=ON
```

### Windows MSVC Link Error

Use `.lib` format:
```bash
cl /EHsc main.cpp /I.\dist\include .\dist\lib\AstraAlgebra.lib
```
