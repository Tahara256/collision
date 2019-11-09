#pragma once

#include <Collider/Collider/ColliderPtr.h>

namespace CollisionAgent {

void collision(const ColliderPtr & collider1, const ColliderPtr & collider2, bool passCollider1, bool passCollider2);

} // namespace TriggerCollisionAgent
