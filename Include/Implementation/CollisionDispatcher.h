#pragma once

#include <Collider/Collider/ColliderPtr.h>

struct Vector3;

/// <summary> �Փˏ������蓖�Ċ֐� </summary>
namespace CollisionDispatcher {

bool intersectBlock(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2);
bool intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2);

} // namespace CollisionDispatcher
