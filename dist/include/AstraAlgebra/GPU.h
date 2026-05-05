// GPU - GPU加速支持，批量运算的时候丢给GPU算，快很多
// 如果你有NVIDIA显卡或者Mac，这个能帮你省不少时间

#pragma once

#include <string>
#include <cstddef>
#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>

namespace AstraAlgebra {

// GPU设备类型
enum class GPUDeviceType {
    CPU,    // CPU（默认，没GPU就用这个）
    CUDA,   // NVIDIA CUDA（N卡用户福音）
    OpenCL, // OpenCL（通用方案）
    Metal,  // Apple Metal（Mac用户看这里）
    Vulkan  // Vulkan（跨平台）
};

// GPU设备信息
struct GPUDeviceInfo {
    std::string name;          // 设备名，比如 "NVIDIA GeForce RTX 3080"
    GPUDeviceType type;        // 类型
    int computeUnits;          // 计算单元数，越多越快
    float clockSpeedMHz;       // 频率（MHz）
    size_t memorySizeMB;       // 显存（MB），爆显存就GG了
    bool isAvailable;          // 能不能用
};

// 输出操作符重载
inline std::ostream& operator<<(std::ostream& os, const GPUDeviceType& type) {
    switch(type) {
        case GPUDeviceType::CPU:    os << "CPU";break;
        case GPUDeviceType::CUDA:   os << "CUDA";break;
        case GPUDeviceType::OpenCL: os << "OpenCL";break;
        case GPUDeviceType::Metal:  os << "Metal";break;
        case GPUDeviceType::Vulkan: os << "Vulkan";break;
        default: os << "Unknown";break;
    }
    return os;
}

inline std::ostream& operator<<(std::ostream& os, const GPUDeviceInfo& info) {
    os << "GPUDeviceInfo{name: " << info.name
       << ", type: " << info.type
       << ", computeUnits: " << info.computeUnits
       << ", clockSpeedMHz: " << info.clockSpeedMHz
       << ", memorySizeMB: " << info.memorySizeMB
       << ", isAvailable: " << (info.isAvailable ? "true" : "false")
       << "}";
    return os;
};

// GPU上下文，管理GPU设备和资源的，单例模式
// 用起来很简单，getInstance()拿到就能用
class GPUContext {
public:
    GPUContext();
    ~GPUContext();
    
    // 初始化，preferredType是首选设备类型
    // 建议用CUDA（N卡）或者Metal（Mac），没的话自动回退到CPU
    bool initialize(GPUDeviceType preferredType = GPUDeviceType::CPU);
    void shutdown();  // 释放资源，程序退出时调用
    
    // 获取设备信息
    bool getDeviceInfo(GPUDeviceInfo& info, int deviceIndex = 0) const;
    int getDeviceCount() const;  // 设备数量
    GPUDeviceType getCurrentDeviceType() const;  // 当前设备类型
    bool isGPUSupported() const;  // 支不支持GPU加速
    bool isDeviceAvailable(GPUDeviceType type) const;  // 某个设备能不能用
    
    void synchronize();  // 同步，等GPU搞完再往下走
    
    static GPUContext& getInstance();  // 拿单例，全局就一个实例
    
private:
    GPUDeviceType m_currentDeviceType;
    bool m_initialized;
    int m_deviceCount;
    
    void detectDevices();  // 检测设备，自动找可用的GPU
    
    GPUContext(const GPUContext&) = delete;
    GPUContext& operator=(const GPUContext&) = delete;
};

// GPU内存管理，分配、释放、拷贝那些
// 跟CUDA的内存管理类似，用起来不复杂
class GPUMemory {
public:
    static void* allocate(size_t size);  // 分配GPU内存
    static void free(void* ptr);  // 释放
    
    // 拷贝数据
    static void copyHostToDevice(void* devicePtr, const void* hostPtr, size_t size);  // CPU->GPU
    static void copyDeviceToHost(void* hostPtr, const void* devicePtr, size_t size);  // GPU->CPU
    static void copyDeviceToDevice(void* dstDevicePtr, const void* srcDevicePtr, size_t size);  // GPU->GPU
    
    static void memset(void* devicePtr, int value, size_t size);  // 初始化GPU内存
};

// 前向声明
class Ray;
class Sphere;
class Plane;
class Triangle;
class Bounds;

// GPU加速的数学函数，批量运算丢给GPU搞
// 适合大量向量/矩阵同时计算的场景，比如粒子系统、骨骼动画
namespace GPU {
    // Vector3操作
    void vectorAdd(const Vector3* a, const Vector3* b, Vector3* result, int count);  // 向量加法
    void vectorSubtract(const Vector3* a, const Vector3* b, Vector3* result, int count);  // 向量减法
    void vectorMultiply(const Vector3* a, const Vector3* b, Vector3* result, int count);  // 分量乘法
    void vectorScale(const Vector3* a, float scalar, Vector3* result, int count);  // 缩放
    void vectorNormalize(Vector3* vectors, int count);  // 归一化
    void dotProduct(const Vector3* a, const Vector3* b, float* result, int count);  // 点积
    void crossProduct(const Vector3* a, const Vector3* b, Vector3* result, int count);  // 叉积
    
    // Vector4操作
    void vectorAdd(const Vector4* a, const Vector4* b, Vector4* result, int count);
    void vectorSubtract(const Vector4* a, const Vector4* b, Vector4* result, int count);
    void vectorMultiply(const Vector4* a, const Vector4* b, Vector4* result, int count);
    void vectorScale(const Vector4* a, float scalar, Vector4* result, int count);
    void vectorNormalize(Vector4* vectors, int count);
    void dotProduct(const Vector4* a, const Vector4* b, float* result, int count);
    
    // 矩阵操作
    void matrixMultiply(const Matrix4x4* a, const Matrix4x4* b, Matrix4x4* result, int count);  // 矩阵乘法
    void matrixTranspose(const Matrix4x4* matrices, Matrix4x4* result, int count);  // 转置
    void matrixVectorMultiply(const Matrix4x4* matrices, const Vector4* vectors, Vector4* result, int count);  // 矩阵乘向量
    
    // 批量射线相交检测（光线追踪、碰撞检测超有用）
    void raySphereIntersect(const Ray* rays, const Sphere* spheres, float* results, int count);  // 射线跟球
    void rayPlaneIntersect(const Ray* rays, const Plane* planes, float* results, int count);  // 射线跟平面
    void rayTriangleIntersect(const Ray* rays, const Triangle* triangles, float* results, int count);  // 射线跟三角形
    void rayAABBIntersect(const Ray* rays, const Bounds* bounds, float* results, int count);  // 射线跟AABB
}

} // namespace AstraAlgebra
