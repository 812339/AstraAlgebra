// AstraAlgebra Benchmark - 性能对比测试
// 对比AstraAlgebra与主流数学库的性能

#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Matrix/Matrix2x2.h>
#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Core/MathUtils.h>

#include <iostream>
#include <chrono>
#include <string>
#include <iomanip>

using namespace AstraAlgebra;
using namespace std::chrono;

// 高精度计时器
class Timer {
public:
    void start() {
        start_time = high_resolution_clock::now();
    }
    
    double elapsed_ms() const {
        auto end_time = high_resolution_clock::now();
        return duration_cast<microseconds>(end_time - start_time).count() / 1000.0;
    }
    
private:
    high_resolution_clock::time_point start_time;
};

// 测试结果结构
struct BenchmarkResult {
    std::string name;
    double time_ms;
    double ops_per_sec;
};

// 防止编译器优化的屏障
volatile float sink_f = 0.0f;
volatile double sink_d = 0.0;

// ========== Vector3 Benchmarks ==========
void benchmark_vector3_operations(int iterations) {
    std::cout << "\n[Vector3 Operations]" << std::endl;
    
    Timer timer;
    
    // 1. 向量加法
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(4.0f, 5.0f, 6.0f);
        Vector3 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = a + b;
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.x + result.y + result.z;
        
        std::cout << "  Addition: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 2. 点积
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(4.0f, 5.0f, 6.0f);
        float result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = a.dot(b);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result;
        
        std::cout << "  Dot Product: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 3. 叉积
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 b(4.0f, 5.0f, 6.0f);
        Vector3 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = a.cross(b);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.x + result.y + result.z;
        
        std::cout << "  Cross Product: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 4. 归一化
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        Vector3 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = a.normalized();
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.x + result.y + result.z;
        
        std::cout << "  Normalize: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 5. 长度计算
    {
        Vector3 a(1.0f, 2.0f, 3.0f);
        float result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = a.length();
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result;
        
        std::cout << "  Length: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 6. Lerp
    {
        Vector3 a(0.0f, 0.0f, 0.0f);
        Vector3 b(10.0f, 10.0f, 10.0f);
        Vector3 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = a.lerp(b, 0.5f);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.x + result.y + result.z;
        
        std::cout << "  Lerp: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
}

// ========== Matrix4x4 Benchmarks ==========
void benchmark_matrix4x4_operations(int iterations) {
    std::cout << "\n[Matrix4x4 Operations]" << std::endl;
    
    Timer timer;
    
    // 1. 矩阵乘法
    {
        Matrix4x4 a = Matrix4x4::identity;
        Matrix4x4 b = Matrix4x4::translation(Vector3(1.0f, 2.0f, 3.0f));
        Matrix4x4 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = a * b;
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.m[0][0] + result.m[1][1] + result.m[2][2];
        
        std::cout << "  Multiplication: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 2. 矩阵求逆
    {
        Matrix4x4 m = Matrix4x4::rotationY(Math::degToRad(45.0f));
        Matrix4x4 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = m.inverse();
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.m[0][0] + result.m[1][1] + result.m[2][2];
        
        std::cout << "  Inverse: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 3. 行列式
    {
        Matrix4x4 m = Matrix4x4::rotationX(Math::degToRad(30.0f));
        float result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = m.determinant();
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result;
        
        std::cout << "  Determinant: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 4. 转置
    {
        Matrix4x4 m = Matrix4x4::scale(Vector3(2.0f, 3.0f, 4.0f));
        Matrix4x4 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = m.transposed();
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.m[0][0] + result.m[1][1] + result.m[2][2];
        
        std::cout << "  Transpose: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 5. 变换向量
    {
        Matrix4x4 m = Matrix4x4::rotationY(Math::degToRad(45.0f));
        Vector3 v(1.0f, 0.0f, 0.0f);
        Vector3 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = m * v;
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.x + result.y + result.z;
        
        std::cout << "  Transform Vector: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
}

// ========== Quaternion Benchmarks ==========
void benchmark_quaternion_operations(int iterations) {
    std::cout << "\n[Quaternion Operations]" << std::endl;
    
    Timer timer;
    
    // 1. 四元数乘法
    {
        Quaternion q1 = Quaternion::identity;
        Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, Math::degToRad(45.0f));
        Quaternion result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = q1 * q2;
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.w;
        
        std::cout << "  Multiplication: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 2. 旋转向量
    {
        Quaternion q = Quaternion::fromAxisAngle(Vector3::up, Math::degToRad(90.0f));
        Vector3 v(1.0f, 0.0f, 0.0f);
        Vector3 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = q.rotate(v);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.x + result.y + result.z;
        
        std::cout << "  Rotate Vector: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 3. Slerp
    {
        Quaternion q1 = Quaternion::identity;
        Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, Math::degToRad(90.0f));
        Quaternion result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = q1.slerp(q2, 0.5f);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.w;
        
        std::cout << "  Slerp: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 4. Fast Slerp
    {
        Quaternion q1 = Quaternion::identity;
        Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, Math::degToRad(90.0f));
        Quaternion result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = q1.fastSlerp(q2, 0.5f);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.w;
        
        std::cout << "  Fast Slerp: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 5. Nlerp
    {
        Quaternion q1 = Quaternion::identity;
        Quaternion q2 = Quaternion::fromAxisAngle(Vector3::up, Math::degToRad(90.0f));
        Quaternion result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = q1.nlerp(q2, 0.5f);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.w;
        
        std::cout << "  Nlerp: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 6. 归一化
    {
        Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
        Quaternion result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = q.normalized();
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.w;
        
        std::cout << "  Normalize: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 7. 共轭
    {
        Quaternion q(1.0f, 2.0f, 3.0f, 4.0f);
        Quaternion result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = q.conjugate();
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.w;
        
        std::cout << "  Conjugate: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
}

// ========== 综合场景测试 ==========
void benchmark_real_world_scenarios(int iterations) {
    std::cout << "\n[Real-World Scenarios]" << std::endl;
    
    Timer timer;
    
    // 1. 相机视图矩阵计算
    {
        Vector3 eye(0.0f, 5.0f, 10.0f);
        Vector3 target(0.0f, 0.0f, 0.0f);
        Vector3 up(0.0f, 1.0f, 0.0f);
        Matrix4x4 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = Matrix4x4::lookAt(eye, target, up);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.m[0][0] + result.m[1][1] + result.m[2][2];
        
        std::cout << "  View Matrix (lookAt): " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 2. 投影矩阵计算
    {
        Matrix4x4 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = Matrix4x4::perspective(Math::degToRad(60.0f), 16.0f/9.0f, 0.1f, 100.0f);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.m[0][0] + result.m[1][1] + result.m[2][2];
        
        std::cout << "  Projection Matrix: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 3. MVP变换
    {
        Matrix4x4 model = Matrix4x4::rotationY(Math::degToRad(45.0f));
        Matrix4x4 view = Matrix4x4::lookAt(Vector3(0, 5, 10), Vector3(0, 0, 0), Vector3::up);
        Matrix4x4 proj = Matrix4x4::perspective(Math::degToRad(60.0f), 16.0f/9.0f, 0.1f, 100.0f);
        Matrix4x4 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = proj * view * model;
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.m[0][0] + result.m[1][1] + result.m[2][2];
        
        std::cout << "  MVP Matrix: " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
    
    // 4. 刚体变换（平移+旋转+缩放）
    {
        Vector3 position(1.0f, 2.0f, 3.0f);
        Quaternion rotation = Quaternion::fromAxisAngle(Vector3::up, Math::degToRad(45.0f));
        Vector3 scale(2.0f, 2.0f, 2.0f);
        Matrix4x4 result;
        
        timer.start();
        for (int i = 0; i < iterations; ++i) {
            result = Matrix4x4::translation(position) *
                     rotation.toRotationMatrix4x4() *
                     Matrix4x4::scale(scale);
        }
        double elapsed = timer.elapsed_ms();
        sink_f = result.m[0][0] + result.m[1][1] + result.m[2][2];
        
        std::cout << "  Transform Matrix (TRS): " << std::fixed << std::setprecision(3) 
                  << elapsed << " ms (" << std::setprecision(0) 
                  << iterations / elapsed * 1000 << " ops/sec)" << std::endl;
    }
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "  AstraAlgebra Benchmark Suite" << std::endl;
    std::cout << "========================================" << std::endl;
    std::cout << "  Compiler: " << 
#if defined(__GNUC__)
        "GCC " << __GNUC__ << "." << __GNUC_MINOR__
#elif defined(_MSC_VER)
        "MSVC " << _MSC_VER
#elif defined(__clang__)
        "Clang " << __clang_major__ << "." << __clang_minor__
#else
        "Unknown"
#endif
        << std::endl;
    std::cout << "  Architecture: " <<
#if defined(__x86_64__) || defined(_M_X64)
        "x86_64"
#elif defined(__i386__) || defined(_M_IX86)
        "x86"
#elif defined(__aarch64__) || defined(_M_ARM64)
        "ARM64"
#else
        "Unknown"
#endif
        << std::endl;
    std::cout << "  SIMD: " <<
#if defined(__AVX2__)
        "AVX2"
#elif defined(__AVX__)
        "AVX"
#elif defined(__SSE4_2__)
        "SSE4.2"
#elif defined(__SSE2__) || defined(_M_X64)
        "SSE2"
#else
        "None"
#endif
        << std::endl;
    std::cout << "========================================" << std::endl;
    
    int iterations = 10000000; // 1000万次迭代
    
    std::cout << "\nRunning benchmarks with " << iterations << " iterations..." << std::endl;
    
    benchmark_vector3_operations(iterations);
    benchmark_matrix4x4_operations(iterations);
    benchmark_quaternion_operations(iterations);
    benchmark_real_world_scenarios(iterations);
    
    std::cout << "\n========================================" << std::endl;
    std::cout << "  Benchmark Complete" << std::endl;
    std::cout << "========================================" << std::endl;
    
    return 0;
}
