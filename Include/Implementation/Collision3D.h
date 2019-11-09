#pragma once

#include <Collider/Face.h>

class Sphere;
class Capsule;
class OBB;
class Segment;
class InfCylinder;
class UniformGridMesh;
struct Vector3;

/// <summary> �Փ˔���֐� </summary>
namespace Collision3D
{

/// <summary> ���̂Ƌ��̂̏Փ˔�������܂��B </summary>
bool intersect(const Sphere & sphere1, const Sphere & sphere2);

/// <summary> ���̂Ƌ��̂̏Փ˔�������܂��B </summary>
bool intersect(const Sphere & sphere1, const Sphere & sphere2, Vector3 * outSphere1HitPos, Vector3 * outSphere2HitPos);

/// <summary> ���̂ƃJ�v�Z���̏Փ˔�������܂��B </summary>
bool intersect(const Sphere & sphere, const Capsule & capsule);

/// <summary> ���̂ƃJ�v�Z���̏Փ˔�������܂��B </summary>
bool intersect(const Sphere & sphere, const Capsule & capsule, Vector3 * outSphereHitPos, Vector3 * outCapsuleHitPos);

/// <summary> ���̂�OBB�̏Փ˔�������܂��B </summary>
bool intersect(const Sphere & sphere, const OBB & obb, Vector3 * outSphereHitPos = nullptr, Vector3 * outOBBHitPos = nullptr);

/// <summary> ���̂ƎO�p�`�̏Փ˔�������܂��B </summary>
bool intersect(const Sphere & sphere, const Triangle & triangle, Vector3 * outSphereHitPos = nullptr, Vector3 * outTriangleHitPos = nullptr);

/// <summary> �J�v�Z���ƃJ�v�Z���̏Փ˔�������܂��B </summary>
bool intersect(const Capsule & capsule1, const Capsule & capsule2);

/// <summary> �J�v�Z���ƃJ�v�Z���̏Փ˔�������܂��B </summary>
bool intersect(const Capsule & capsule1, const Capsule & capsule2, Vector3 * outCapsule1HitPos, Vector3 * outCapsule2HitPos);

/// <summary> �J�v�Z����OBB�̏Փ˔�������܂��B </summary>
bool intersect(const Capsule & capsule, const OBB & obb);

/// <summary> �J�v�Z����OBB�̏Փ˔�������܂��B </summary>
bool intersect(const Capsule & capsule, const OBB & obb, Vector3 * outCapsuleExtrusion);

/// <summary> �J�v�Z����OBB�̏Փ˔�������܂��B </summary>
bool intersect(const Capsule & capsule, const OBB & obb, Vector3 * outCapsuleHitPos, Vector3 * outOBBHitPos);

/// <summary> �J�v�Z���ƎO�p�`�̏Փ˔�������܂��B </summary>
bool intersect(const Capsule & capsule, const Triangle & triangle, Vector3 * outCapsuleHitPos = nullptr, Vector3 * outTriangleHitPos = nullptr);

/// <summary> OBB��OBB�̏Փ˔�������܂��B </summary>
bool intersect(const OBB & obb1, const OBB & obb2);

/// <summary> OBB��OBB�̏Փ˔�������܂��B </summary>
bool intersect(const OBB & obb1, const OBB & obb2, Vector3 * outOBB1Extrusion);

/// <summary> OBB��OBB�̏Փ˔�������܂��B </summary>
bool intersect(const OBB & obb1, const OBB & obb2, Vector3 * outOBB1HitPos, Vector3 * outOBB2HitPos);

/// <summary> OBB�ƎO�p�`�̏Փ˔�������܂��B </summary>
bool intersect(const OBB & obb, const Triangle & triangle);

/// <summary> OBB�ƎO�p�`�̏Փ˔�������܂��B </summary>
bool intersect(const OBB & obb, const Triangle & triangle, Vector3 * outOBBHitPos, Vector3 * outTriangleHitPos);

/// <summary> �����Ƌ��̂̏Փ˔�������܂��B </summary>
bool intersect(const Segment & segment, const Sphere & sphere, float * outHitDistance = nullptr);

/// <summary> �����Ɖ~���̏Փ˔�������܂��B </summary>
bool intersect(const Segment & segment, const InfCylinder & cylinder, float * outHitDistance = nullptr);

/// <summary> �����ƃJ�v�Z���̏Փ˔�������܂��B </summary>
bool intersect(const Segment & segment, const Capsule & capsule, float * outHitDistance = nullptr);

/// <summary> ������OBB�̏Փ˔�������܂��B </summary>
bool intersect(const Segment & segment, const OBB & obb, float * outHitDistance = nullptr);

/// <summary> �����ƎO�p�`�̏Փ˔�������܂��B </summary>
bool intersect(const Segment & segment, const Triangle & triangle, float * outHitDistance = nullptr);

} // namespace Collision3D
