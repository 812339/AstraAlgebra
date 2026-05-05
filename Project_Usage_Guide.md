# AstraAlgebra Project Usage Guide

This document explains how to use the AstraAlgebra math library in your projects, including building, integration, and common issue resolution.

## Getting the Library

### Option 1: Building from Source

```bash
mkdir build && cd build
cmake .. -DCMAKE_BUILD_TYPE=Release -G Ninja
cmake --build . --config Release
```

### Option 2: Using Precompiled Distribution

Distribution package structure:
```
dist/
├── include/AstraAlgebra/    # Header files
└── lib/
    ├── libAstraAlgebra.a    # Static library
    └── cmake/AstraAlgebra/  # CMake configuration files
```

## Project Integration

### CMake Subproject (Recommended)

Add AstraAlgebra as a subproject to your CMake project:

```cmake
add_subdirectory(path/to/AstraAlgebra)
target_link_libraries(YourProject PRIVATE AstraAlgebra)
```

### find_package (After Installation)

If AstraAlgebra has been installed to your system:

```cmake
find_package(AstraAlgebra REQUIRED)
target_link_libraries(YourProject PRIVATE AstraAlgebra::AstraAlgebra)
```

### Manual Path Specification

```cmake
target_include_directories(YourProject PRIVATE /path/to/AstraAlgebra/dist/include)
target_link_libraries(YourProject PRIVATE /path/to/AstraAlgebra/dist/lib/libAstraAlgebra.a)
```

### GCC/Clang Command Line

```bash
g++ main.cpp -I./dist/include -L./dist/lib -lAstraAlgebra -o myprogram
```

## Quick Start

```cpp
#include <AstraAlgebra/AstraAlgebra.h>
#include <iostream>

using namespace AstraAlgebra;

int main() {
    // Vector operations
    Vector3 v(1.0f, 2.0f, 3.0f);
    std::cout << "Length: " << v.length() << std::endl;

    // Quaternion rotation
    Quaternion rot = Quaternion::fromAxisAngle(Vector3::up, 90.0f, true);
    Vector3 rotated = rot * Vector3::forward;

    // Matrix transformation
    Matrix4x4 transform = Matrix4x4::translation(Vector3(5.0f, 0.0f, 0.0f));
    Vector3 result = transform * v;

    // Collision detection
    Sphere sphere(Vector3::zero, 1.0f);
    Ray ray(Vector3::zero, Vector3::forward);
    float t = ray.intersect(sphere);

    return 0;
}
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

### Windows MSVC Link Error

Use `.lib` format library file:
```bash
cl /EHsc main.cpp /I.\dist\include .\dist\lib\AstraAlgebra.lib
```

### SIMD Not Enabled

Add optimization flags during compilation:
```bash
# GCC/Clang
-mavx2 -mavx -msse2

# Or use CMake auto-detection
cmake .. -DENABLE_SIMD=ON
```

### Compilation Speed Optimization

Use Release mode for better performance:
```bash
cmake .. -DCMAKE_BUILD_TYPE=Release
```

## Performance Tips

1. **Small data operations**: Use library functions directly, SIMD will auto-optimize
2. **Large data (10k+ elements)**: Use GPU acceleration module
3. **Compile-time computable expressions**: Use `constexpr` feature for zero runtime overhead
4. **Rotation operations**: Prefer quaternions over matrices for better performance
