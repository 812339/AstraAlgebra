// GPU.cpp - GPU加速支持实现
// 批量运算丢给GPU算，快很多

#include <AstraAlgebra/Core/GPU.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Triangle.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <cstdlib>  // 包含std::malloc和std::free的定义

// CUDA支持检测
#ifdef __CUDA_ARCH__
#include <cuda_runtime.h>
#endif

// 预处理器宏，用于检查编译时是否支持CUDA
#ifdef __CUDACC__
#define ASTRAALGEBRA_CUDA_SUPPORTED
#endif

// Windows平台特定头文件
#if defined(_WIN32) || defined(_WIN64)
#include <windows.h>
#endif

namespace AstraAlgebra {

// GPUContext构造函数
GPUContext::GPUContext() : m_currentDeviceType(GPUDeviceType::CPU), m_initialized(false), m_deviceCount(1) {
    // 初始化设备数量为1（CPU设备），避免在构造函数中调用可能失败的检测
    m_deviceCount = 1;
}

// GPUContext析构函数
GPUContext::~GPUContext() {
    shutdown();
}

// 检测可用的GPU设备
void GPUContext::detectDevices() {
    m_deviceCount = 1; // 默认至少有一个CPU设备
    
    // 检测CUDA设备
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    int cudaDeviceCount = 0;
    cudaGetDeviceCount(&cudaDeviceCount);
    if (cudaDeviceCount > 0) {
        m_deviceCount += cudaDeviceCount;
    }
#endif
    
    // 在这里可以添加其他GPU设备类型的检测
    // 例如OpenCL、Metal、Vulkan等
}

// 初始化GPU上下文
bool GPUContext::initialize(GPUDeviceType preferredType) {
    if (m_initialized) {
        return true;
    }
    
    // 检查首选设备类型是否可用
    if (preferredType == GPUDeviceType::CPU || !isDeviceAvailable(preferredType)) {
        // 如果首选设备不可用，使用CPU设备
        m_currentDeviceType = GPUDeviceType::CPU;
    } else {
        m_currentDeviceType = preferredType;
    }
    
    m_initialized = true;
    return true;
}

// 释放GPU上下文
void GPUContext::shutdown() {
    if (m_initialized) {
        m_initialized = false;
        m_currentDeviceType = GPUDeviceType::CPU;
    }
}

// 获取设备信息
bool GPUContext::getDeviceInfo(GPUDeviceInfo& info, int deviceIndex) const {
    if (deviceIndex < 0 || deviceIndex >= m_deviceCount) {
        return false;
    }
    
    // 处理CPU设备
    if (deviceIndex == 0) {
        info.name = "CPU Device";
        info.type = GPUDeviceType::CPU;
        info.computeUnits = 1;
        info.clockSpeedMHz = 0.0f;
        info.memorySizeMB = 0;
        info.isAvailable = true;
        return true;
    }
    
    // 处理CUDA设备
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    int cudaDeviceIndex = deviceIndex - 1;
    cudaDeviceProp prop;
    cudaError_t error = cudaGetDeviceProperties(&prop, cudaDeviceIndex);
    if (error == cudaSuccess) {
        info.name = prop.name;
        info.type = GPUDeviceType::CUDA;
        info.computeUnits = prop.multiProcessorCount;
        info.clockSpeedMHz = static_cast<float>(prop.clockRate) / 1000.0f; // 转换为MHz
        info.memorySizeMB = static_cast<size_t>(prop.totalGlobalMem) / (1024 * 1024); // 转换为MB
        info.isAvailable = true;
        return true;
    }
#endif
    
    // 处理其他GPU设备类型
    // 例如OpenCL、Metal、Vulkan等
    
    return false;
}

// 获取设备数量
int GPUContext::getDeviceCount() const {
    return m_deviceCount;
}

// 获取当前设备类型
GPUDeviceType GPUContext::getCurrentDeviceType() const {
    return m_currentDeviceType;
}

// 检查GPU加速是否可用
bool GPUContext::isGPUSupported() const {
    // 如果设备数量大于1，说明有GPU设备可用
    return m_deviceCount > 1;
}

// 检查特定设备类型是否可用
bool GPUContext::isDeviceAvailable(GPUDeviceType type) const {
    if (type == GPUDeviceType::CPU) {
        return true; // CPU设备始终可用
    }
    
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    if (type == GPUDeviceType::CUDA) {
        int cudaDeviceCount = 0;
        cudaGetDeviceCount(&cudaDeviceCount);
        return cudaDeviceCount > 0;
    }
#endif
    
    // 检查其他GPU设备类型
    // 例如OpenCL、Metal、Vulkan等
    
    return false;
}

// 同步操作
void GPUContext::synchronize() {
    // CPU设备不需要同步
}

// 获取GPUContext单例
GPUContext& GPUContext::getInstance() {
    static GPUContext instance;
    return instance;
}

// GPUMemory类的内存分配
void* GPUMemory::allocate(size_t size) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA内存分配
    void* gpuPtr = nullptr;
    cudaMalloc(&gpuPtr, size);
    return gpuPtr;
#else
    // CPU fallback: 使用标准内存分配
    return std::malloc(size);
#endif
}

// GPUMemory类的内存释放
void GPUMemory::free(void* ptr) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA内存释放
    cudaFree(ptr);
#else
    // CPU fallback: 使用标准内存释放
    std::free(ptr);
#endif
}

// 内存拷贝：主机到设备
void GPUMemory::copyHostToDevice(void* devicePtr, const void* hostPtr, size_t size) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA内存拷贝
    cudaMemcpy(devicePtr, hostPtr, size, cudaMemcpyHostToDevice);
#else
    // CPU fallback: 直接内存拷贝
    memcpy(devicePtr, hostPtr, size);
#endif
}

// 内存拷贝：设备到主机
void GPUMemory::copyDeviceToHost(void* hostPtr, const void* devicePtr, size_t size) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA内存拷贝
    cudaMemcpy(hostPtr, devicePtr, size, cudaMemcpyDeviceToHost);
#else
    // CPU fallback: 直接内存拷贝
    memcpy(hostPtr, devicePtr, size);
#endif
}

// 内存拷贝：设备到设备
void GPUMemory::copyDeviceToDevice(void* dstDevicePtr, const void* srcDevicePtr, size_t size) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA内存拷贝
    cudaMemcpy(dstDevicePtr, srcDevicePtr, size, cudaMemcpyDeviceToDevice);
#else
    // CPU fallback: 直接内存拷贝
    memcpy(dstDevicePtr, srcDevicePtr, size);
#endif
}

// 内存初始化
void GPUMemory::memset(void* devicePtr, int value, size_t size) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA内存初始化
    cudaMemset(devicePtr, value, size);
#else
    // CPU fallback: 使用标准memset（使用全局作用域限定符避免递归调用）
    ::memset(devicePtr, value, size);
#endif
}

// CUDA内核：向量加法
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVectorAddKernel(const Vector4* a, const Vector4* b, Vector4* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i].x = a[i].x + b[i].x;
        result[i].y = a[i].y + b[i].y;
        result[i].z = a[i].z + b[i].z;
        result[i].w = a[i].w + b[i].w;
    }
}
#endif

// GPU命名空间的向量加法实现
void GPU::vectorAdd(const Vector4* a, const Vector4* b, Vector4* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector4* d_a = nullptr;
    Vector4* d_b = nullptr;
    Vector4* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector4));
    cudaMalloc(&d_b, count * sizeof(Vector4));
    cudaMalloc(&d_result, count * sizeof(Vector4));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVectorAddKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector4), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] + b[i];
    }
#endif
}

// CUDA内核：向量减法
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVectorSubtractKernel(const Vector4* a, const Vector4* b, Vector4* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i].x = a[i].x - b[i].x;
        result[i].y = a[i].y - b[i].y;
        result[i].z = a[i].z - b[i].z;
        result[i].w = a[i].w - b[i].w;
    }
}
#endif

// GPU命名空间的向量减法实现
void GPU::vectorSubtract(const Vector4* a, const Vector4* b, Vector4* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector4* d_a = nullptr;
    Vector4* d_b = nullptr;
    Vector4* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector4));
    cudaMalloc(&d_b, count * sizeof(Vector4));
    cudaMalloc(&d_result, count * sizeof(Vector4));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVectorSubtractKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector4), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] - b[i];
    }
#endif
}

// CUDA内核：向量分量乘法
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVectorMultiplyKernel(const Vector4* a, const Vector4* b, Vector4* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i].x = a[i].x * b[i].x;
        result[i].y = a[i].y * b[i].y;
        result[i].z = a[i].z * b[i].z;
        result[i].w = a[i].w * b[i].w;
    }
}
#endif

// GPU命名空间的向量乘法实现（分量级乘法）
void GPU::vectorMultiply(const Vector4* a, const Vector4* b, Vector4* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector4* d_a = nullptr;
    Vector4* d_b = nullptr;
    Vector4* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector4));
    cudaMalloc(&d_b, count * sizeof(Vector4));
    cudaMalloc(&d_result, count * sizeof(Vector4));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVectorMultiplyKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector4), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = Vector4(
            a[i].x * b[i].x,
            a[i].y * b[i].y,
            a[i].z * b[i].z,
            a[i].w * b[i].w
        );
    }
#endif
}

// CUDA内核：向量缩放
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVectorScaleKernel(const Vector4* a, float scalar, Vector4* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i].x = a[i].x * scalar;
        result[i].y = a[i].y * scalar;
        result[i].z = a[i].z * scalar;
        result[i].w = a[i].w * scalar;
    }
}
#endif

// GPU命名空间的向量缩放实现
void GPU::vectorScale(const Vector4* a, float scalar, Vector4* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector4* d_a = nullptr;
    Vector4* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector4));
    cudaMalloc(&d_result, count * sizeof(Vector4));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVectorScaleKernel<<<gridSize, blockSize>>>(d_a, scalar, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector4), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] * scalar;
    }
#endif
}

// CUDA内核：向量归一化
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVectorNormalizeKernel(Vector4* vectors, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        float x = vectors[i].x;
        float y = vectors[i].y;
        float z = vectors[i].z;
        float w = vectors[i].w;
        
        float length = sqrtf(x * x + y * y + z * z + w * w);
        if (length > 1e-6f) {
            float invLength = 1.0f / length;
            vectors[i].x = x * invLength;
            vectors[i].y = y * invLength;
            vectors[i].z = z * invLength;
            vectors[i].w = w * invLength;
        }
    }
}
#endif

// GPU命名空间的向量归一化实现
void GPU::vectorNormalize(Vector4* vectors, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector4* d_vectors = nullptr;
    
    cudaMalloc(&d_vectors, count * sizeof(Vector4));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_vectors, vectors, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVectorNormalizeKernel<<<gridSize, blockSize>>>(d_vectors, count);
    
    // 拷贝结果回主机
    cudaMemcpy(vectors, d_vectors, count * sizeof(Vector4), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_vectors);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        vectors[i].normalize();
    }
#endif
}

// CUDA内核：矩阵乘法
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaMatrixMultiplyKernel(const Matrix4x4* a, const Matrix4x4* b, Matrix4x4* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        const Matrix4x4& matA = a[i];
        const Matrix4x4& matB = b[i];
        Matrix4x4& matResult = result[i];
        
        // 4x4矩阵乘法的具体实现
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                float sum = 0.0f;
                for (int k = 0; k < 4; ++k) {
                    sum += matA.m[row][k] * matB.m[k][col];
                }
                matResult.m[row][col] = sum;
            }
        }
    }
}
#endif

// GPU命名空间的矩阵乘法实现
void GPU::matrixMultiply(const Matrix4x4* a, const Matrix4x4* b, Matrix4x4* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Matrix4x4* d_a = nullptr;
    Matrix4x4* d_b = nullptr;
    Matrix4x4* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Matrix4x4));
    cudaMalloc(&d_b, count * sizeof(Matrix4x4));
    cudaMalloc(&d_result, count * sizeof(Matrix4x4));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Matrix4x4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Matrix4x4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaMatrixMultiplyKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Matrix4x4), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] * b[i];
    }
#endif
}

// CUDA内核：矩阵转置
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaMatrixTransposeKernel(const Matrix4x4* matrices, Matrix4x4* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        const Matrix4x4& mat = matrices[i];
        Matrix4x4& matResult = result[i];
        
        // 4x4矩阵转置的具体实现
        for (int row = 0; row < 4; ++row) {
            for (int col = 0; col < 4; ++col) {
                matResult.m[row][col] = mat.m[col][row];
            }
        }
    }
}
#endif

// GPU命名空间的矩阵转置实现
void GPU::matrixTranspose(const Matrix4x4* matrices, Matrix4x4* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Matrix4x4* d_matrices = nullptr;
    Matrix4x4* d_result = nullptr;
    
    cudaMalloc(&d_matrices, count * sizeof(Matrix4x4));
    cudaMalloc(&d_result, count * sizeof(Matrix4x4));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_matrices, matrices, count * sizeof(Matrix4x4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaMatrixTransposeKernel<<<gridSize, blockSize>>>(d_matrices, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Matrix4x4), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_matrices);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = matrices[i].transposed();
    }
#endif
}

// CUDA内核：向量点积
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaDotProductKernel(const Vector4* a, const Vector4* b, float* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i] = a[i].x * b[i].x + a[i].y * b[i].y + a[i].z * b[i].z + a[i].w * b[i].w;
    }
}
#endif

// GPU命名空间的点积实现
void GPU::dotProduct(const Vector4* a, const Vector4* b, float* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector4* d_a = nullptr;
    Vector4* d_b = nullptr;
    float* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector4));
    cudaMalloc(&d_b, count * sizeof(Vector4));
    cudaMalloc(&d_result, count * sizeof(float));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaDotProductKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(float), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i].dot(b[i]);
    }
#endif
}

// CUDA内核：向量叉积
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaCrossProductKernel(const Vector3* a, const Vector3* b, Vector3* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        float ax = a[i].x;
        float ay = a[i].y;
        float az = a[i].z;
        float bx = b[i].x;
        float by = b[i].y;
        float bz = b[i].z;
        
        result[i].x = ay * bz - az * by;
        result[i].y = az * bx - ax * bz;
        result[i].z = ax * by - ay * bx;
    }
}
#endif

// GPU命名空间的叉积实现
void GPU::crossProduct(const Vector3* a, const Vector3* b, Vector3* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector3* d_a = nullptr;
    Vector3* d_b = nullptr;
    Vector3* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector3));
    cudaMalloc(&d_b, count * sizeof(Vector3));
    cudaMalloc(&d_result, count * sizeof(Vector3));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaCrossProductKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector3), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = Vector3::cross(a[i], b[i]);
    }
#endif
}

// CUDA内核：Vector3加法
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVector3AddKernel(const Vector3* a, const Vector3* b, Vector3* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i].x = a[i].x + b[i].x;
        result[i].y = a[i].y + b[i].y;
        result[i].z = a[i].z + b[i].z;
    }
}
#endif

// GPU命名空间的Vector3加法实现
void GPU::vectorAdd(const Vector3* a, const Vector3* b, Vector3* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector3* d_a = nullptr;
    Vector3* d_b = nullptr;
    Vector3* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector3));
    cudaMalloc(&d_b, count * sizeof(Vector3));
    cudaMalloc(&d_result, count * sizeof(Vector3));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVector3AddKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector3), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] + b[i];
    }
#endif
}

// CUDA内核：Vector3减法
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVector3SubtractKernel(const Vector3* a, const Vector3* b, Vector3* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i].x = a[i].x - b[i].x;
        result[i].y = a[i].y - b[i].y;
        result[i].z = a[i].z - b[i].z;
    }
}
#endif

// GPU命名空间的Vector3减法实现
void GPU::vectorSubtract(const Vector3* a, const Vector3* b, Vector3* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector3* d_a = nullptr;
    Vector3* d_b = nullptr;
    Vector3* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector3));
    cudaMalloc(&d_b, count * sizeof(Vector3));
    cudaMalloc(&d_result, count * sizeof(Vector3));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVector3SubtractKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector3), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] - b[i];
    }
#endif
}

// CUDA内核：Vector3分量乘法
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVector3MultiplyKernel(const Vector3* a, const Vector3* b, Vector3* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i].x = a[i].x * b[i].x;
        result[i].y = a[i].y * b[i].y;
        result[i].z = a[i].z * b[i].z;
    }
}
#endif

// GPU命名空间的Vector3分量乘法实现
void GPU::vectorMultiply(const Vector3* a, const Vector3* b, Vector3* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector3* d_a = nullptr;
    Vector3* d_b = nullptr;
    Vector3* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector3));
    cudaMalloc(&d_b, count * sizeof(Vector3));
    cudaMalloc(&d_result, count * sizeof(Vector3));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVector3MultiplyKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector3), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = Vector3(
            a[i].x * b[i].x,
            a[i].y * b[i].y,
            a[i].z * b[i].z
        );
    }
#endif
}

// CUDA内核：Vector3缩放
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVector3ScaleKernel(const Vector3* a, float scalar, Vector3* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i].x = a[i].x * scalar;
        result[i].y = a[i].y * scalar;
        result[i].z = a[i].z * scalar;
    }
}
#endif

// GPU命名空间的Vector3缩放实现
void GPU::vectorScale(const Vector3* a, float scalar, Vector3* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector3* d_a = nullptr;
    Vector3* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector3));
    cudaMalloc(&d_result, count * sizeof(Vector3));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVector3ScaleKernel<<<gridSize, blockSize>>>(d_a, scalar, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector3), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i] * scalar;
    }
#endif
}

// CUDA内核：Vector3归一化
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVector3NormalizeKernel(Vector3* vectors, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        float x = vectors[i].x;
        float y = vectors[i].y;
        float z = vectors[i].z;
        
        float length = sqrtf(x * x + y * y + z * z);
        if (length > 1e-6f) {
            float invLength = 1.0f / length;
            vectors[i].x = x * invLength;
            vectors[i].y = y * invLength;
            vectors[i].z = z * invLength;
        }
    }
}
#endif

// GPU命名空间的Vector3归一化实现
void GPU::vectorNormalize(Vector3* vectors, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector3* d_vectors = nullptr;
    
    cudaMalloc(&d_vectors, count * sizeof(Vector3));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_vectors, vectors, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVector3NormalizeKernel<<<gridSize, blockSize>>>(d_vectors, count);
    
    // 拷贝结果回主机
    cudaMemcpy(vectors, d_vectors, count * sizeof(Vector3), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_vectors);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        vectors[i].normalize();
    }
#endif
}

// CUDA内核：Vector3点积
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaVector3DotProductKernel(const Vector3* a, const Vector3* b, float* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        result[i] = a[i].x * b[i].x + a[i].y * b[i].y + a[i].z * b[i].z;
    }
}
#endif

// GPU命名空间的Vector3点积实现
void GPU::dotProduct(const Vector3* a, const Vector3* b, float* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Vector3* d_a = nullptr;
    Vector3* d_b = nullptr;
    float* d_result = nullptr;
    
    cudaMalloc(&d_a, count * sizeof(Vector3));
    cudaMalloc(&d_b, count * sizeof(Vector3));
    cudaMalloc(&d_result, count * sizeof(float));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_a, a, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    cudaMemcpy(d_b, b, count * sizeof(Vector3), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaVector3DotProductKernel<<<gridSize, blockSize>>>(d_a, d_b, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(float), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_a);
    cudaFree(d_b);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = a[i].dot(b[i]);
    }
#endif
}

// CUDA内核：矩阵向量乘法（Matrix4x4 * Vector4）
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaMatrixVectorMultiplyKernel(const Matrix4x4* matrices, const Vector4* vectors, Vector4* result, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        const Matrix4x4& mat = matrices[i];
        const Vector4& vec = vectors[i];
        Vector4& res = result[i];
        
        res.x = mat.m[0][0] * vec.x + mat.m[0][1] * vec.y + mat.m[0][2] * vec.z + mat.m[0][3] * vec.w;
        res.y = mat.m[1][0] * vec.x + mat.m[1][1] * vec.y + mat.m[1][2] * vec.z + mat.m[1][3] * vec.w;
        res.z = mat.m[2][0] * vec.x + mat.m[2][1] * vec.y + mat.m[2][2] * vec.z + mat.m[2][3] * vec.w;
        res.w = mat.m[3][0] * vec.x + mat.m[3][1] * vec.y + mat.m[3][2] * vec.z + mat.m[3][3] * vec.w;
    }
}
#endif

// GPU命名空间的矩阵向量乘法实现
void GPU::matrixVectorMultiply(const Matrix4x4* matrices, const Vector4* vectors, Vector4* result, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Matrix4x4* d_matrices = nullptr;
    Vector4* d_vectors = nullptr;
    Vector4* d_result = nullptr;
    
    cudaMalloc(&d_matrices, count * sizeof(Matrix4x4));
    cudaMalloc(&d_vectors, count * sizeof(Vector4));
    cudaMalloc(&d_result, count * sizeof(Vector4));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_matrices, matrices, count * sizeof(Matrix4x4), cudaMemcpyHostToDevice);
    cudaMemcpy(d_vectors, vectors, count * sizeof(Vector4), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaMatrixVectorMultiplyKernel<<<gridSize, blockSize>>>(d_matrices, d_vectors, d_result, count);
    
    // 拷贝结果回主机
    cudaMemcpy(result, d_result, count * sizeof(Vector4), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_matrices);
    cudaFree(d_vectors);
    cudaFree(d_result);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        result[i] = matrices[i] * vectors[i];
    }
#endif
}

// CUDA内核：射线与球体批量相交检测
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaRaySphereIntersectKernel(const Ray* rays, const Sphere* spheres, float* results, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        const Ray& ray = rays[i];
        const Sphere& sphere = spheres[i];
        
        Vector3 oc = ray.origin - sphere.center;
        float a = ray.direction.dot(ray.direction);
        float b = 2.0f * ray.direction.dot(oc);
        float c = oc.dot(oc) - (sphere.radius * sphere.radius);
        float discriminant = b * b - 4.0f * a * c;
        
        if (discriminant < 0.0f) {
            results[i] = -1.0f; // 不相交
        } else {
            float sqrtDiscriminant = sqrtf(discriminant);
            float t1 = (-b - sqrtDiscriminant) / (2.0f * a);
            float t2 = (-b + sqrtDiscriminant) / (2.0f * a);
            
            // 只返回最近的有效交点
            if (t1 > 0.0f) {
                results[i] = t1;
            } else if (t2 > 0.0f) {
                results[i] = t2;
            } else {
                results[i] = -1.0f; // 不相交
            }
        }
    }
}
#endif

// GPU命名空间的射线与球体批量相交检测实现
void GPU::raySphereIntersect(const Ray* rays, const Sphere* spheres, float* results, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Ray* d_rays = nullptr;
    Sphere* d_spheres = nullptr;
    float* d_results = nullptr;
    
    cudaMalloc(&d_rays, count * sizeof(Ray));
    cudaMalloc(&d_spheres, count * sizeof(Sphere));
    cudaMalloc(&d_results, count * sizeof(float));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_rays, rays, count * sizeof(Ray), cudaMemcpyHostToDevice);
    cudaMemcpy(d_spheres, spheres, count * sizeof(Sphere), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaRaySphereIntersectKernel<<<gridSize, blockSize>>>(d_rays, d_spheres, d_results, count);
    
    // 拷贝结果回主机
    cudaMemcpy(results, d_results, count * sizeof(float), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_rays);
    cudaFree(d_spheres);
    cudaFree(d_results);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        results[i] = rays[i].intersect(spheres[i]);
    }
#endif
}

// CUDA内核：射线与平面批量相交检测
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaRayPlaneIntersectKernel(const Ray* rays, const Plane* planes, float* results, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        const Ray& ray = rays[i];
        const Plane& plane = planes[i];
        
        // 使用平面的a、b、c、d成员变量
        Vector3 planeNormal(plane.a, plane.b, plane.c);
        float denom = ray.direction.dot(planeNormal);
        
        if (fabs(denom) < 1e-6f) {
            results[i] = -1.0f; // 射线与平面平行，不相交
        } else {
            float t = -(ray.origin.dot(planeNormal) + plane.d) / denom;
            if (t > 0.0f) {
                results[i] = t;
            } else {
                results[i] = -1.0f; // 不相交
            }
        }
    }
}
#endif

// GPU命名空间的射线与平面批量相交检测实现
void GPU::rayPlaneIntersect(const Ray* rays, const Plane* planes, float* results, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Ray* d_rays = nullptr;
    Plane* d_planes = nullptr;
    float* d_results = nullptr;
    
    cudaMalloc(&d_rays, count * sizeof(Ray));
    cudaMalloc(&d_planes, count * sizeof(Plane));
    cudaMalloc(&d_results, count * sizeof(float));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_rays, rays, count * sizeof(Ray), cudaMemcpyHostToDevice);
    cudaMemcpy(d_planes, planes, count * sizeof(Plane), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaRayPlaneIntersectKernel<<<gridSize, blockSize>>>(d_rays, d_planes, d_results, count);
    
    // 拷贝结果回主机
    cudaMemcpy(results, d_results, count * sizeof(float), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_rays);
    cudaFree(d_planes);
    cudaFree(d_results);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        results[i] = rays[i].intersect(planes[i]);
    }
#endif
}

// CUDA内核：射线与三角形批量相交检测（Möller-Trumbore算法）
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaRayTriangleIntersectKernel(const Ray* rays, const Triangle* triangles, float* results, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        const Ray& ray = rays[i];
        const Triangle& triangle = triangles[i];
        
        // 注意：Triangle类的顶点成员是a、b、c，不是v0、v1、v2
        const Vector3& v0 = triangle.a;
        const Vector3& v1 = triangle.b;
        const Vector3& v2 = triangle.c;
        
        Vector3 edge1 = v1 - v0;
        Vector3 edge2 = v2 - v0;
        Vector3 h = ray.direction.cross(edge2);
        float a = edge1.dot(h);
        
        if (fabs(a) < 1e-6f) {
            results[i] = -1.0f; // 射线与三角形平行
            return;
        }
        
        float f = 1.0f / a;
        Vector3 s = ray.origin - v0;
        float u = f * s.dot(h);
        
        if (u < 0.0f || u > 1.0f) {
            results[i] = -1.0f;
            return;
        }
        
        Vector3 q = s.cross(edge1);
        float v = f * ray.direction.dot(q);
        
        if (v < 0.0f || u + v > 1.0f) {
            results[i] = -1.0f;
            return;
        }
        
        float t = f * edge2.dot(q);
        
        if (t > 0.0f) {
            results[i] = t;
        } else {
            results[i] = -1.0f;
        }
    }
}
#endif

// GPU命名空间的射线与三角形批量相交检测实现
void GPU::rayTriangleIntersect(const Ray* rays, const Triangle* triangles, float* results, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Ray* d_rays = nullptr;
    Triangle* d_triangles = nullptr;
    float* d_results = nullptr;
    
    cudaMalloc(&d_rays, count * sizeof(Ray));
    cudaMalloc(&d_triangles, count * sizeof(Triangle));
    cudaMalloc(&d_results, count * sizeof(float));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_rays, rays, count * sizeof(Ray), cudaMemcpyHostToDevice);
    cudaMemcpy(d_triangles, triangles, count * sizeof(Triangle), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaRayTriangleIntersectKernel<<<gridSize, blockSize>>>(d_rays, d_triangles, d_results, count);
    
    // 拷贝结果回主机
    cudaMemcpy(results, d_results, count * sizeof(float), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_rays);
    cudaFree(d_triangles);
    cudaFree(d_results);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        results[i] = rays[i].intersect(triangles[i]);
    }
#endif
}

// CUDA内核：射线与AABB批量相交检测（Slab方法）
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
__global__ void cudaRayAABBIntersectKernel(const Ray* rays, const Bounds* bounds, float* results, int count) {
    int i = blockIdx.x * blockDim.x + threadIdx.x;
    if (i < count) {
        const Ray& ray = rays[i];
        const Bounds& box = bounds[i];
        
        Vector3 invDir = Vector3(1.0f / ray.direction.x, 1.0f / ray.direction.y, 1.0f / ray.direction.z);
        
        Vector3 tMinVec = (box.min - ray.origin) * invDir;
        Vector3 tMaxVec = (box.max - ray.origin) * invDir;
        
        Vector3 tMin = Vector3(fminf(tMinVec.x, tMaxVec.x), fminf(tMinVec.y, tMaxVec.y), fminf(tMinVec.z, tMaxVec.z));
        Vector3 tMax = Vector3(fmaxf(tMinVec.x, tMaxVec.x), fmaxf(tMinVec.y, tMaxVec.y), fmaxf(tMinVec.z, tMaxVec.z));
        
        float tMinMax = fmaxf(fmaxf(tMin.x, tMin.y), tMin.z);
        float tMaxMin = fminf(fminf(tMax.x, tMax.y), tMax.z);
        
        if (tMinMax <= tMaxMin && tMaxMin > 0.0f) {
            results[i] = tMinMax;
        } else {
            results[i] = -1.0f;
        }
    }
}
#endif

// GPU命名空间的射线与AABB批量相交检测实现
void GPU::rayAABBIntersect(const Ray* rays, const Bounds* bounds, float* results, int count) {
#ifdef ASTRAALGEBRA_CUDA_SUPPORTED
    // 使用CUDA加速实现
    int blockSize = 256;
    int gridSize = (count + blockSize - 1) / blockSize;
    
    // 分配GPU内存
    Ray* d_rays = nullptr;
    Bounds* d_bounds = nullptr;
    float* d_results = nullptr;
    
    cudaMalloc(&d_rays, count * sizeof(Ray));
    cudaMalloc(&d_bounds, count * sizeof(Bounds));
    cudaMalloc(&d_results, count * sizeof(float));
    
    // 拷贝数据到GPU
    cudaMemcpy(d_rays, rays, count * sizeof(Ray), cudaMemcpyHostToDevice);
    cudaMemcpy(d_bounds, bounds, count * sizeof(Bounds), cudaMemcpyHostToDevice);
    
    // 执行CUDA内核
    cudaRayAABBIntersectKernel<<<gridSize, blockSize>>>(d_rays, d_bounds, d_results, count);
    
    // 拷贝结果回主机
    cudaMemcpy(results, d_results, count * sizeof(float), cudaMemcpyDeviceToHost);
    
    // 释放GPU内存
    cudaFree(d_rays);
    cudaFree(d_bounds);
    cudaFree(d_results);
#else
    // CPU fallback实现
    for (int i = 0; i < count; ++i) {
        results[i] = rays[i].intersect(bounds[i]);
    }
#endif
}

} // namespace AstraAlgebra