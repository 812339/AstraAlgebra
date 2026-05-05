# AstraAlgebra - High-Performance Game Engine Math Library

AstraAlgebra is a C++ math library designed for game engines and graphics applications. It includes vector, matrix, quaternion operations, collision detection (GJK/EPA), GPU acceleration, numerical methods, noise generation, and color space conversion.

## Features

| Module | Description |
|--------|-------------|
| **Vectors** | Vector2 / Vector3 / Vector4 / Vector3d (double precision) |
| **Matrices** | Matrix2x2 / Matrix3x3 / Matrix4x4 / Matrix4x4d (double precision) |
| **Quaternions** | Rotation, Slerp/Nlerp/FastSlerp, fromToRotation, lookRotation |
| **Geometry** | Ray / Sphere / Plane / Bounds / Triangle / Capsule / Cone / Cylinder |
| **Collision Detection** | GJK + EPA algorithm, supports all 7 geometry combinations |
| **Numerical Methods** | 5 interpolation methods, 4 numerical integrators, root finding, gradient descent |
| **Matrix Decomposition** | LU / QR / Cholesky / SVD / Eigenvalue / Polar decomposition |
| **GPU Acceleration** | CUDA batch vector/matrix operations, batch ray intersection |
| **SIMD Optimization** | SSE2 / AVX / AVX2 auto-detection |
| **Utilities** | Color space (RGB/HSV/HSL/CMYK), Perlin noise, random number (multiple distributions) |
| **Transform** | Hierarchical transform system with parent-child relationships |
| **constexpr** | Core types support compile-time computation, zero runtime overhead |

## Quick Start

### Requirements

- C++17 or higher
- CMake 3.10 or higher
- Compilers: MSVC / GCC / Clang

### Build

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTS=ON -G Ninja
cmake --build . --config Release
```

**CMake Options:**
- `ENABLE_SIMD=ON` — Enable SIMD optimization (enabled by default, auto-detects AVX2/AVX/SSE2)
- `BUILD_TESTS=ON` — Build unit tests
- `BUILD_EXAMPLES=ON` — Build example programs

### Run Tests

```bash
./AstraAlgebraTests
# 232 tests, 100% pass rate
```

### Install

```bash
cmake --install . --prefix /your/install/path
```

## Usage Examples

### Vector Operations

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

### Quaternion Rotation

```cpp
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>

using namespace AstraAlgebra;

int main() {
    // Rotate 90 degrees around Y axis
    Quaternion rot = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);

    // Rotate a vector
    Vector3 forward = Vector3::forward;
    Vector3 rotated = rot * forward;

    // Interpolation
    Quaternion q1 = Quaternion::identity;
    Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, 180.0f, true);
    Quaternion slerped = q1.slerp(q2, 0.5f);

    return 0;
}
```

### Matrix Transformations

```cpp
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>

using namespace AstraAlgebra;

int main() {
    Matrix4x4 translation = Matrix4x4::translation(Vector3(5.0f, 0.0f, 0.0f));
    Matrix4x4 rotation = Matrix4x4::rotationY(90.0f, true);
    Matrix4x4 scale = Matrix4x4::scale(Vector3(2.0f, 2.0f, 2.0f));

    // Combine transformations
    Matrix4x4 transform = translation * rotation * scale;

    // Transform a point
    Vector3 point(1.0f, 0.0f, 0.0f);
    Vector3 result = transform * point;

    return 0;
}
```

### Collision Detection (GJK)

```cpp
#include <AstraAlgebra/Geometry/GJK.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Capsule.h>

using namespace AstraAlgebra;

int main() {
    Sphere sphere(Vector3(0.0f, 0.0f, 0.0f), 1.0f);
    Capsule capsule(Vector3(-2.0f, 0.0f, 0.0f), Vector3(2.0f, 0.0f, 0.0f), 0.5f);

    // Quick test
    bool hit = gjkIntersect(sphere, capsule);

    // Get detailed collision info
    GJKResult result = gjkIntersectionDetails(sphere, capsule);
    if (result.intersect) {
        // result.normal      - collision normal
        // result.penetration - penetration depth
        // result.contactPoint - contact point
    }

    return 0;
}
```

### Transform Hierarchy

```cpp
#include <AstraAlgebra/Core/Transform.h>

using namespace AstraAlgebra;

int main() {
    Transform parent;
    parent.setPosition(Vector3(1.0f, 0.0f, 0.0f));

    Transform child;
    child.setPosition(Vector3(2.0f, 0.0f, 0.0f));
    child.setParent(&parent);

    // Get world position
    Vector3 worldPos = child.getWorldPosition();
    Matrix4x4 worldMatrix = child.getWorldMatrix();

    return 0;
}
```

### Numerical Methods

```cpp
#include <AstraAlgebra/Numerics/NumericalMethods.h>

using namespace AstraAlgebra;
using namespace AstraAlgebra::Numerics;

int main() {
    // Interpolation
    std::vector<Vector3> points = { ... };
    Vector3 interpolated = catmullRom(points, 0.5f);

    // Numerical integration (4th-order Runge-Kutta)
    auto derivative = [](const Vector3& state) { return ...; };
    Vector3 next = rungeKutta4(current, derivative, dt);

    // Root finding (Newton's method)
    auto func = [](float x) { return x * x - 2.0f; };
    auto deriv = [](float x) { return 2.0f * x; };
    float root = newtonRaphson(func, deriv, 1.0f);

    return 0;
}
```

### GPU Batch Operations

```cpp
#include <AstraAlgebra/Core/GPU.h>

using namespace AstraAlgebra;

int main() {
    const int count = 10000;
    std::vector<Vector3> a(count), b(count), result(count);

    // GPU-accelerated batch operations
    GPU::vectorAdd(a.data(), b.data(), result.data(), count);
    GPU::vectorScale(a.data(), 2.0f, result.data(), count);

    // Batch ray intersection
    std::vector<Ray> rays(count);
    std::vector<Sphere> spheres(count);
    std::vector<float> distances(count);
    GPU::raySphereIntersect(rays.data(), spheres.data(), distances.data(), count);

    return 0;
}
```

## Using in CMake Projects

### Option 1: Subproject

```cmake
add_subdirectory(AstraAlgebra)
target_link_libraries(YourProject PRIVATE AstraAlgebra)
```

### Option 2: Installed Library

```cmake
find_package(AstraAlgebra REQUIRED)
target_link_libraries(YourProject PRIVATE AstraAlgebra::AstraAlgebra)
```

### Option 3: Manual Linking

```bash
g++ main.cpp -I./dist/include -L./dist/lib -lAstraAlgebra -o myprogram
```

## Project Structure

```
AstraAlgebra/
├── include/AstraAlgebra/     # Header files (public API)
│   ├── Core/                 # Core utilities (GPU, SIMD, Serialization, Transform)
│   ├── Geometry/             # Geometry and collision detection (GJK)
│   ├── Matrix/               # Matrices (2x2 ~ 4x4, including double precision)
│   ├── Numerics/             # Numerical methods (interpolation, integration, colors, random)
│   ├── Quaternion/           # Quaternions
│   ├── Vector/               # Vectors (2D/3D/4D, including double precision)
│   └── AstraAlgebra.h        # Unified header
├── src/                      # Source implementation
├── examples/                 # Example programs
├── test/                     # Unit tests and benchmarks
└── CMakeLists.txt            # Build configuration
```

## Performance

| Optimization Level | Description |
|--------------------|-------------|
| **Algorithm Optimization** | Quaternion vector rotation (90% faster than traditional), algebraic matrix inversion (78% faster) |
| **SIMD** | Auto-detects CPU instruction set, enables AVX2/AVX/SSE2 optimizations |
| **GPU** | CUDA batch operations, suitable for data parallel processing with 10k+ elements |
| **constexpr** | Core computations can be done at compile time, zero runtime overhead |

## Documentation

- [API Usage Guide](API_Usage_Guide.md) — Detailed API reference with code examples
- [Project Usage Guide](Project_Usage_Guide.md) — How to integrate AstraAlgebra into your project

## License

MIT License

---

**AstraAlgebra** — A high-performance math library for game engines
