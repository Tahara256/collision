#pragma once

#include <Collider/Face.h>

class Sphere;
class Capsule;
class OBB;
class Segment;
class InfCylinder;
class UniformGridMesh;
struct Vector3;

/// <summary> 衝突判定関数 </summary>
namespace Collision3D
{

/// <summary> 球体と球体の衝突判定をします。 </summary>
bool intersect(const Sphere & sphere1, const Sphere & sphere2);

/// <summary> 球体と球体の衝突判定をします。 </summary>
bool intersect(const Sphere & sphere1, const Sphere & sphere2, Vector3 * outSphere1HitPos, Vector3 * outSphere2HitPos);

/// <summary> 球体とカプセルの衝突判定をします。 </summary>
bool intersect(const Sphere & sphere, const Capsule & capsule);

/// <summary> 球体とカプセルの衝突判定をします。 </summary>
bool intersect(const Sphere & sphere, const Capsule & capsule, Vector3 * outSphereHitPos, Vector3 * outCapsuleHitPos);

/// <summary> 球体とOBBの衝突判定をします。 </summary>
bool intersect(const Sphere & sphere, const OBB & obb, Vector3 * outSphereHitPos = nullptr, Vector3 * outOBBHitPos = nullptr);

/// <summary> 球体と三角形の衝突判定をします。 </summary>
bool intersect(const Sphere & sphere, const Triangle & triangle, Vector3 * outSphereHitPos = nullptr, Vector3 * outTriangleHitPos = nullptr);

/// <summary> カプセルとカプセルの衝突判定をします。 </summary>
bool intersect(const Capsule & capsule1, const Capsule & capsule2);

/// <summary> カプセルとカプセルの衝突判定をします。 </summary>
bool intersect(const Capsule & capsule1, const Capsule & capsule2, Vector3 * outCapsule1HitPos, Vector3 * outCapsule2HitPos);

/// <summary> カプセルとOBBの衝突判定をします。 </summary>
bool intersect(const Capsule & capsule, const OBB & obb);

/// <summary> カプセルとOBBの衝突判定をします。 </summary>
bool intersect(const Capsule & capsule, const OBB & obb, Vector3 * outCapsuleExtrusion);

/// <summary> カプセルとOBBの衝突判定をします。 </summary>
bool intersect(const Capsule & capsule, const OBB & obb, Vector3 * outCapsuleHitPos, Vector3 * outOBBHitPos);

/// <summary> カプセルと三角形の衝突判定をします。 </summary>
bool intersect(const Capsule & capsule, const Triangle & triangle, Vector3 * outCapsuleHitPos = nullptr, Vector3 * outTriangleHitPos = nullptr);

/// <summary> OBBとOBBの衝突判定をします。 </summary>
bool intersect(const OBB & obb1, const OBB & obb2);

/// <summary> OBBとOBBの衝突判定をします。 </summary>
bool intersect(const OBB & obb1, const OBB & obb2, Vector3 * outOBB1Extrusion);

/// <summary> OBBとOBBの衝突判定をします。 </summary>
bool intersect(const OBB & obb1, const OBB & obb2, Vector3 * outOBB1HitPos, Vector3 * outOBB2HitPos);

/// <summary> OBBと三角形の衝突判定をします。 </summary>
bool intersect(const OBB & obb, const Triangle & triangle);

/// <summary> OBBと三角形の衝突判定をします。 </summary>
bool intersect(const OBB & obb, const Triangle & triangle, Vector3 * outOBBHitPos, Vector3 * outTriangleHitPos);

/// <summary> 線分と球体の衝突判定をします。 </summary>
bool intersect(const Segment & segment, const Sphere & sphere, float * outHitDistance = nullptr);

/// <summary> 線分と円柱の衝突判定をします。 </summary>
bool intersect(const Segment & segment, const InfCylinder & cylinder, float * outHitDistance = nullptr);

/// <summary> 線分とカプセルの衝突判定をします。 </summary>
bool intersect(const Segment & segment, const Capsule & capsule, float * outHitDistance = nullptr);

/// <summary> 線分とOBBの衝突判定をします。 </summary>
bool intersect(const Segment & segment, const OBB & obb, float * outHitDistance = nullptr);

/// <summary> 線分と三角形の衝突判定をします。 </summary>
bool intersect(const Segment & segment, const Triangle & triangle, float * outHitDistance = nullptr);

} // namespace Collision3D
