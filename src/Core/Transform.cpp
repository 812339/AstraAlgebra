// Transform.cpp - 变换系统实现
// 位置旋转缩放那些，游戏里天天用

#include <AstraAlgebra/Core/Transform.h>

namespace AstraAlgebra {

Transform::Transform()
    : m_position(Vector3::zero)
    , m_rotation(Quaternion::Identity())
    , m_scale(Vector3::one)
    , m_parent(nullptr)
    , m_localMatrixDirty(true)
    , m_worldMatrixDirty(true) {
}

Transform::Transform(const Vector3& position, const Quaternion& rotation, const Vector3& scale)
    : m_position(position)
    , m_rotation(rotation)
    , m_scale(scale)
    , m_parent(nullptr)
    , m_localMatrixDirty(true)
    , m_worldMatrixDirty(true) {
}

Transform::Transform(const Vector3& position, const Vector3& rotationEuler, const Vector3& scale)
    : m_position(position)
    , m_rotation(Quaternion::fromEulerAngles(rotationEuler))
    , m_scale(scale)
    , m_parent(nullptr)
    , m_localMatrixDirty(true)
    , m_worldMatrixDirty(true) {
}

Transform::Transform(const Matrix4x4& transformMatrix) {
    m_parent = nullptr;
    setTransformMatrix(transformMatrix);
}

void Transform::reset() {
    m_position = Vector3::zero;
    m_rotation = Quaternion::Identity();
    m_scale = Vector3::one;
    m_parent = nullptr;
    markMatricesDirty();
}

void Transform::setPosition(const Vector3& position) {
    m_position = position;
    markMatricesDirty();
}

void Transform::setPosition(float x, float y, float z) {
    m_position.set(x, y, z);
    markMatricesDirty();
}

const Vector3& Transform::getPosition() const {
    return m_position;
}

void Transform::translate(const Vector3& translation) {
    m_position += translation;
    markMatricesDirty();
}

void Transform::translate(float x, float y, float z) {
    m_position += Vector3(x, y, z);
    markMatricesDirty();
}

void Transform::setRotation(const Quaternion& rotation) {
    m_rotation = rotation;
    markMatricesDirty();
}

void Transform::setRotation(const Vector3& eulerAngles) {
    m_rotation = Quaternion::fromEulerAngles(eulerAngles);
    markMatricesDirty();
}

void Transform::setRotation(float pitch, float yaw, float roll) {
    m_rotation = Quaternion::fromEulerAngles(Vector3(pitch, yaw, roll));
    markMatricesDirty();
}

const Quaternion& Transform::getRotation() const {
    return m_rotation;
}

Vector3 Transform::getEulerAngles() const {
    return m_rotation.toEulerAngles();
}

void Transform::rotate(const Quaternion& rotation) {
    m_rotation = rotation * m_rotation;
    m_rotation.normalize();
    markMatricesDirty();
}

void Transform::rotate(const Vector3& eulerAngles) {
    Quaternion rotation = Quaternion::fromEulerAngles(eulerAngles);
    m_rotation = rotation * m_rotation;
    m_rotation.normalize();
    markMatricesDirty();
}

void Transform::rotateAround(const Vector3& point, const Quaternion& rotation) {
    Vector3 direction = m_position - point;
    direction = rotation * direction;
    m_position = point + direction;
    m_rotation = rotation * m_rotation;
    m_rotation.normalize();
    markMatricesDirty();
}

void Transform::setScale(const Vector3& scale) {
    m_scale = scale;
    markMatricesDirty();
}

void Transform::setScale(float scale) {
    m_scale.set(scale, scale, scale);
    markMatricesDirty();
}

void Transform::setScale(float x, float y, float z) {
    m_scale.set(x, y, z);
    markMatricesDirty();
}

const Vector3& Transform::getScale() const {
    return m_scale;
}

void Transform::scale(const Vector3& scaleFactor) {
    m_scale *= scaleFactor;
    markMatricesDirty();
}

void Transform::scale(float factor) {
    m_scale *= factor;
    markMatricesDirty();
}

void Transform::setParent(Transform* parent) {
    m_parent = parent;
    markMatricesDirty();
}

Transform* Transform::getParent() const {
    return m_parent;
}

const Matrix4x4& Transform::getLocalMatrix() const {
    if (m_localMatrixDirty) {
        updateLocalMatrix();
    }
    return m_localMatrix;
}

const Matrix4x4& Transform::getWorldMatrix() const {
    if (m_worldMatrixDirty || (m_parent && m_parent->m_worldMatrixDirty)) {
        updateWorldMatrix();
    }
    return m_worldMatrix;
}

void Transform::updateLocalMatrix() const {
    // 计算局部变换矩阵：缩放 -> 旋转 -> 平移
    Matrix4x4 scaleMat = Matrix4x4::scale(m_scale);
    Matrix4x4 rotationMat = m_rotation.toRotationMatrix4x4();
    Matrix4x4 translationMat = Matrix4x4::translation(m_position);
    
    m_localMatrix = translationMat * rotationMat * scaleMat;
    m_localMatrixDirty = false;
    m_worldMatrixDirty = true;  // 局部矩阵变化会影响世界矩阵
}

void Transform::updateWorldMatrix() const {
    if (m_localMatrixDirty) {
        updateLocalMatrix();
    }
    
    if (m_parent) {
        m_worldMatrix = m_parent->getWorldMatrix() * m_localMatrix;
    } else {
        m_worldMatrix = m_localMatrix;
    }
    
    m_worldMatrixDirty = false;
}

Vector3 Transform::localToWorld(const Vector3& point) const {
    return getWorldMatrix() * point;
}

Vector3 Transform::worldToLocal(const Vector3& point) const {
    return getWorldMatrix().inverse() * point;
}

Vector3 Transform::localToWorldDirection(const Vector3& direction) const {
    // 方向向量转换不需要考虑平移
    Matrix4x4 rotationScaleMat = getWorldMatrix();
    rotationScaleMat.setTranslation(Vector3::zero);
    return rotationScaleMat * direction;
}

Vector3 Transform::worldToLocalDirection(const Vector3& direction) const {
    // 方向向量转换不需要考虑平移
    Matrix4x4 rotationScaleMat = getWorldMatrix().inverse();
    rotationScaleMat.setTranslation(Vector3::zero);
    return rotationScaleMat * direction;
}

Vector3 Transform::getForward() const {
    return m_rotation * Vector3::forward;
}

Vector3 Transform::getRight() const {
    return m_rotation * Vector3::right;
}

Vector3 Transform::getUp() const {
    return m_rotation * Vector3::up;
}

void Transform::setTransformMatrix(const Matrix4x4& matrix) {
    // 从矩阵中分解出位置、旋转和缩放
    matrix.decompose(m_position, m_rotation, m_scale);
    markMatricesDirty();
}

Transform Transform::lookAt(const Vector3& eye, const Vector3& target, const Vector3& up) {
    Transform result;
    result.m_position = eye;
    result.m_rotation = Quaternion::lookRotation(target - eye, up);
    result.m_scale = Vector3::one;
    result.markMatricesDirty();
    return result;
}

bool Transform::operator==(const Transform& other) const {
    return m_position == other.m_position &&
           m_rotation == other.m_rotation &&
           m_scale == other.m_scale &&
           m_parent == other.m_parent;
}

bool Transform::operator!=(const Transform& other) const {
    return !(*this == other);
}

void Transform::markMatricesDirty() const {
    m_localMatrixDirty = true;
    m_worldMatrixDirty = true;
}

} // namespace AstraAlgebra