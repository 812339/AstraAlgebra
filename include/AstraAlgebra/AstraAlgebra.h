// AstraAlgebra - 游戏引擎数学库，一个头文件全搞定
// 包含这个就能用所有功能了

#pragma once

// 核心模块
#include <AstraAlgebra/Core/MathUtils.h>
#include <AstraAlgebra/Core/SIMD.h>
#include <AstraAlgebra/Core/GPU.h>
#include <AstraAlgebra/Core/Serialization.h>
#include <AstraAlgebra/Core/Transform.h>

// 向量
#include <AstraAlgebra/Vector/Vector2.h>
#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Vector/Vector4.h>
#include <AstraAlgebra/Vector/Vector3d.h>

// 矩阵
#include <AstraAlgebra/Matrix/Matrix2x2.h>
#include <AstraAlgebra/Matrix/Matrix3x3.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>
#include <AstraAlgebra/Matrix/Matrix4x4d.h>

// 四元数
#include <AstraAlgebra/Quaternion/Quaternion.h>

// 几何体
#include <AstraAlgebra/Geometry/Ray.h>
#include <AstraAlgebra/Geometry/Plane.h>
#include <AstraAlgebra/Geometry/Sphere.h>
#include <AstraAlgebra/Geometry/Bounds.h>
#include <AstraAlgebra/Geometry/Triangle.h>
#include <AstraAlgebra/Geometry/Noise.h>
#include <AstraAlgebra/Geometry/Cylinder.h>
#include <AstraAlgebra/Geometry/Cone.h>
#include <AstraAlgebra/Geometry/Capsule.h>
#include <AstraAlgebra/Geometry/Line.h>

// 数值方法
#include <AstraAlgebra/Numerics/MatrixDecomposition.h>
#include <AstraAlgebra/Numerics/ColorSpace.h>
#include <AstraAlgebra/Numerics/Random.h>
