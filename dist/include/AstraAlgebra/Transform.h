// Transform - 变换系统，位置旋转缩放那些，游戏里天天用

#pragma once

#include <AstraAlgebra/Vector/Vector3.h>
#include <AstraAlgebra/Quaternion/Quaternion.h>
#include <AstraAlgebra/Matrix/Matrix4x4.h>

namespace AstraAlgebra {

// Transform类，管理物体的位置、旋转、缩放，还有父子关系
class Transform {
private:
    Vector3 m_position;              // 位置
    Quaternion m_rotation;           // 旋转，用四元数存
    Vector3 m_scale;                 // 缩放
    Transform* m_parent;             // 父节点
    
    mutable Matrix4x4 m_localMatrix;      // 局部矩阵，缓存的
    mutable Matrix4x4 m_worldMatrix;      // 世界矩阵，也是缓存
    mutable bool m_localMatrixDirty;      // 脏标记，变了才重新算
    mutable bool m_worldMatrixDirty;      // 世界矩阵的脏标记

public:
    // 构造函数
    Transform();
    Transform(const Vector3& position, const Quaternion& rotation = Quaternion::Identity(), const Vector3& scale = Vector3::one);
    Transform(const Vector3& position, const Vector3& rotationEuler, const Vector3& scale = Vector3::one);
    Transform(const Matrix4x4& transformMatrix);
    
    // 重置
    void reset();
    
    // 位置
    void setPosition(const Vector3& position);
    void setPosition(float x, float y, float z);
    const Vector3& getPosition() const;
    void translate(const Vector3& translation);
    void translate(float x, float y, float z);
    
    // 旋转
    void setRotation(const Quaternion& rotation);
    void setRotation(const Vector3& eulerAngles);
    void setRotation(float pitch, float yaw, float roll);
    const Quaternion& getRotation() const;
    Vector3 getEulerAngles() const;
    void rotate(const Quaternion& rotation);
    void rotate(const Vector3& eulerAngles);
    void rotateAround(const Vector3& point, const Quaternion& rotation);
    
    // 缩放
    void setScale(const Vector3& scale);
    void setScale(float scale);
    void setScale(float x, float y, float z);
    const Vector3& getScale() const;
    void scale(const Vector3& scaleFactor);
    void scale(float factor);
    
    // 父子关系
    void setParent(Transform* parent);
    Transform* getParent() const;
    
    // 矩阵，带缓存的，脏了才更新
    const Matrix4x4& getLocalMatrix() const;
    const Matrix4x4& getWorldMatrix() const;
    void updateLocalMatrix() const;
    void updateWorldMatrix() const;
    
    // 空间转换
    Vector3 localToWorld(const Vector3& point) const;
    Vector3 worldToLocal(const Vector3& point) const;
    Vector3 localToWorldDirection(const Vector3& direction) const;
    Vector3 worldToLocalDirection(const Vector3& direction) const;
    
    // 方向向量
    Vector3 getForward() const;
    Vector3 getRight() const;
    Vector3 getUp() const;
    
    // 直接设置矩阵
    void setTransformMatrix(const Matrix4x4& matrix);
    
    // 静态方法，看向某个目标
    static Transform lookAt(const Vector3& eye, const Vector3& target, const Vector3& up = Vector3::up);
    
    // 比较
    bool operator==(const Transform& other) const;
    bool operator!=(const Transform& other) const;
    
private:
    // 标记矩阵脏了
    void markMatricesDirty() const;
};

} // namespace AstraAlgebra
