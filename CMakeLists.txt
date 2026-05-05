# CMakeLists.txt - 构建配置
# 用来编译整个项目的

# CMake最低版本要求
cmake_minimum_required(VERSION 3.10)

# 项目名称、版本和语言
project(AstraAlgebra VERSION 0.1.0 LANGUAGES CXX)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

### 选项配置
option(BUILD_SHARED_LIBS "构建动态库" OFF)
option(BUILD_TESTS "构建测试" ON)
option(ENABLE_SIMD "启用SIMD优化" ON)

# 定义库的源文件
set(CORE_SOURCES
    src/Core/MathUtils.cpp
    src/Core/GPU.cpp
    src/Core/Serialization.cpp
)

set(VECTOR_SOURCES
    src/Vector/Vector2.cpp
    src/Vector/Vector3.cpp
    src/Vector/Vector4.cpp
)

set(MATRIX_SOURCES
    src/Matrix/Matrix2x2.cpp
    src/Matrix/Matrix3x3.cpp
    src/Matrix/Matrix4x4.cpp
    src/Matrix/Matrix4x4d.cpp
)

set(QUATERNION_SOURCES
    src/Quaternion/Quaternion.cpp
)

set(GEOMETRY_SOURCES
    src/Geometry/Ray.cpp
    src/Geometry/Plane.cpp
    src/Geometry/Sphere.cpp
    src/Geometry/Bounds.cpp
    src/Geometry/Triangle.cpp
    src/Geometry/Noise.cpp
    src/Geometry/Cylinder.cpp
    src/Geometry/Cone.cpp
    src/Geometry/Capsule.cpp
    src/Geometry/Line.cpp
)

set(TRANSFORM_SOURCES
    src/Core/Transform.cpp
)

set(NUMERICS_SOURCES
    src/Numerics/Random.cpp
)

# 合并所有源文件
set(SOURCES
    ${CORE_SOURCES}
    ${VECTOR_SOURCES}
    ${MATRIX_SOURCES}
    ${QUATERNION_SOURCES}
    ${GEOMETRY_SOURCES}
    ${TRANSFORM_SOURCES}
    ${NUMERICS_SOURCES}
)

# 定义头文件目录
set(INCLUDE_DIRS
    ${CMAKE_CURRENT_SOURCE_DIR}/include
)

# 添加头文件目录
include_directories(${INCLUDE_DIRS})

# 创建库
add_library(AstraAlgebra ${SOURCES})

# 设置库的头文件
target_include_directories(AstraAlgebra
    PUBLIC
        $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
        $<INSTALL_INTERFACE:include>
)

# 启用SIMD优化（如果支持）
if(ENABLE_SIMD)
    if(MSVC)
        # 检测编译器支持的SIMD级别
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("/arch:AVX2" COMPILER_SUPPORTS_AVX2)
        check_cxx_compiler_flag("/arch:AVX" COMPILER_SUPPORTS_AVX)
        
        if(COMPILER_SUPPORTS_AVX2)
            target_compile_options(AstraAlgebra PRIVATE /arch:AVX2)
            target_compile_definitions(AstraAlgebra PRIVATE __AVX2__)
            set(SIMD_FLAGS "/arch:AVX2")
            set(SIMD_DEFS "__AVX2__")
            message(STATUS "Enabling AVX2 SIMD optimizations")
        elseif(COMPILER_SUPPORTS_AVX)
            target_compile_options(AstraAlgebra PRIVATE /arch:AVX)
            target_compile_definitions(AstraAlgebra PRIVATE __AVX__)
            set(SIMD_FLAGS "/arch:AVX")
            set(SIMD_DEFS "__AVX__")
            message(STATUS "Enabling AVX SIMD optimizations")
        else()
            target_compile_options(AstraAlgebra PRIVATE /arch:SSE2)
            target_compile_definitions(AstraAlgebra PRIVATE __SSE2__)
            set(SIMD_FLAGS "/arch:SSE2")
            set(SIMD_DEFS "__SSE2__")
            message(STATUS "Enabling SSE2 SIMD optimizations")
        endif()
    else()
        include(CheckCXXCompilerFlag)
        check_cxx_compiler_flag("-mavx2" COMPILER_SUPPORTS_AVX2)
        check_cxx_compiler_flag("-mavx" COMPILER_SUPPORTS_AVX)
        
        if(COMPILER_SUPPORTS_AVX2)
            target_compile_options(AstraAlgebra PRIVATE -mavx2)
            target_compile_definitions(AstraAlgebra PRIVATE __AVX2__)
            set(SIMD_FLAGS "-mavx2")
            set(SIMD_DEFS "__AVX2__")
            message(STATUS "Enabling AVX2 SIMD optimizations")
        elseif(COMPILER_SUPPORTS_AVX)
            target_compile_options(AstraAlgebra PRIVATE -mavx)
            target_compile_definitions(AstraAlgebra PRIVATE __AVX__)
            set(SIMD_FLAGS "-mavx")
            set(SIMD_DEFS "__AVX__")
            message(STATUS "Enabling AVX SIMD optimizations")
        else()
            target_compile_options(AstraAlgebra PRIVATE -msse2)
            target_compile_definitions(AstraAlgebra PRIVATE __SSE2__)
            set(SIMD_FLAGS "-msse2")
            set(SIMD_DEFS "__SSE2__")
            message(STATUS "Enabling SSE2 SIMD optimizations")
        endif()
    endif()
endif()

# 函数：为目标应用SIMD优化
function(apply_simd_optimizations target)
    if(ENABLE_SIMD AND SIMD_FLAGS)
        target_compile_options(${target} PRIVATE ${SIMD_FLAGS})
        target_compile_definitions(${target} PRIVATE ${SIMD_DEFS})
    endif()
endfunction()

# 添加示例程序构建选项
option(BUILD_EXAMPLES "构建示例程序" ON)

# 构建示例程序
if(BUILD_EXAMPLES)
    add_executable(AstraAlgebraExample examples/BasicUsage.cpp)
    target_include_directories(AstraAlgebraExample PRIVATE include)
    target_link_libraries(AstraAlgebraExample PRIVATE AstraAlgebra)
    
    # 添加新的演示程序
    add_executable(AstraAlgebraDemo examples/RunDemo.cpp)
    target_include_directories(AstraAlgebraDemo PRIVATE include)
    target_link_libraries(AstraAlgebraDemo PRIVATE AstraAlgebra)
    
    # 新增功能演示程序
    add_executable(NewFeaturesDemo examples/NewFeaturesDemo.cpp)
    target_include_directories(NewFeaturesDemo PRIVATE include)
    target_link_libraries(NewFeaturesDemo PRIVATE AstraAlgebra)
endif()

# 构建测试
if(BUILD_TESTS)
    add_executable(AstraAlgebraTests test/AstraAlgebraTests.cpp)
    target_include_directories(AstraAlgebraTests PRIVATE include)
    target_link_libraries(AstraAlgebraTests PRIVATE AstraAlgebra)
    apply_simd_optimizations(AstraAlgebraTests)
    
    # 构建Benchmark
    add_executable(AstraAlgebraBenchmark test/AstraAlgebraBenchmark.cpp)
    target_include_directories(AstraAlgebraBenchmark PRIVATE include)
    target_link_libraries(AstraAlgebraBenchmark PRIVATE AstraAlgebra)
    apply_simd_optimizations(AstraAlgebraBenchmark)
endif()

# 安装配置
install(TARGETS AstraAlgebra
    EXPORT AstraAlgebraTargets
    ARCHIVE DESTINATION lib
    LIBRARY DESTINATION lib
    RUNTIME DESTINATION bin
    INCLUDES DESTINATION include
)

# 安装头文件
install(DIRECTORY include/AstraAlgebra
    DESTINATION include
    FILES_MATCHING PATTERN "*.h"
)

# 生成导出文件
install(EXPORT AstraAlgebraTargets
    FILE AstraAlgebraTargets.cmake
    NAMESPACE AstraAlgebra::
    DESTINATION lib/cmake/AstraAlgebra
)

# 生成版本配置文件
include(CMakePackageConfigHelpers)
write_basic_package_version_file(
    "AstraAlgebraConfigVersion.cmake"
    VERSION ${PROJECT_VERSION}
    COMPATIBILITY SameMajorVersion
)

# 生成配置文件
configure_file(AstraAlgebraConfig.cmake.in AstraAlgebraConfig.cmake @ONLY)

# 安装配置文件
install(FILES
    "${CMAKE_CURRENT_BINARY_DIR}/AstraAlgebraConfig.cmake"
    "${CMAKE_CURRENT_BINARY_DIR}/AstraAlgebraConfigVersion.cmake"
    DESTINATION lib/cmake/AstraAlgebra
)

# 导出构建树
export(EXPORT AstraAlgebraTargets
    FILE "${CMAKE_CURRENT_BINARY_DIR}/AstraAlgebraTargets.cmake"
    NAMESPACE AstraAlgebra::
)

# 注册到构建树
export(PACKAGE AstraAlgebra)

# 生成pkg-config文件
configure_file(AstraAlgebra.pc.in AstraAlgebra.pc @ONLY)
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/AstraAlgebra.pc"
    DESTINATION lib/pkgconfig
)

# 添加编译特性
target_compile_features(AstraAlgebra PUBLIC cxx_std_17)

# 添加编译警告
if(MSVC)
    target_compile_options(AstraAlgebra PRIVATE /W4 /WX-)
else()
    target_compile_options(AstraAlgebra PRIVATE -Wall -Wextra -Wno-unused-parameter -Wno-unused-variable)
endif()

# 添加优化标志
if(CMAKE_BUILD_TYPE STREQUAL "Release")
    if(MSVC)
        target_compile_options(AstraAlgebra PRIVATE /O2 /GL)
        target_link_options(AstraAlgebra PRIVATE /LTCG)
    else()
        target_compile_options(AstraAlgebra PRIVATE -O3 -march=native -flto)
        target_link_options(AstraAlgebra PRIVATE -flto)
    endif()
endif()

# 导出头文件列表以便其他项目使用
set(ASTRA_ALGEBRA_HEADERS
    include/AstraAlgebra/AstraAlgebra.h
    include/AstraAlgebra/Core/MathUtils.h
    include/AstraAlgebra/Core/GPU.h
    include/AstraAlgebra/Core/Serialization.h
    include/AstraAlgebra/Core/Transform.h
    include/AstraAlgebra/Vector/Vector2.h
    include/AstraAlgebra/Vector/Vector3.h
    include/AstraAlgebra/Vector/Vector4.h
    include/AstraAlgebra/Vector/Vector3d.h
    include/AstraAlgebra/Matrix/Matrix2x2.h
    include/AstraAlgebra/Matrix/Matrix3x3.h
    include/AstraAlgebra/Matrix/Matrix4x4.h
    include/AstraAlgebra/Matrix/Matrix4x4d.h
    include/AstraAlgebra/Quaternion/Quaternion.h
    include/AstraAlgebra/Geometry/Ray.h
    include/AstraAlgebra/Geometry/Plane.h
    include/AstraAlgebra/Geometry/Sphere.h
    include/AstraAlgebra/Geometry/Bounds.h
    include/AstraAlgebra/Geometry/Triangle.h
    include/AstraAlgebra/Geometry/Noise.h
    include/AstraAlgebra/Geometry/Cylinder.h
    include/AstraAlgebra/Geometry/Cone.h
    include/AstraAlgebra/Geometry/Capsule.h
    include/AstraAlgebra/Geometry/Line.h
    include/AstraAlgebra/Numerics/MatrixDecomposition.h
    include/AstraAlgebra/Numerics/ColorSpace.h
    include/AstraAlgebra/Numerics/Random.h
)

# 安装所有头文件
install(FILES ${ASTRA_ALGEBRA_HEADERS}
    DESTINATION include/AstraAlgebra
)
