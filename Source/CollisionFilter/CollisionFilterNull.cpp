#include <Collider/CollisionFilter/CollisionFilterNull.h>

std::pair<bool, bool> CollisionFilterNull::filter(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	return { true, true };
}
