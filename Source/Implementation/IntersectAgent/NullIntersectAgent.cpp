#include <Collider/Implementation/IntersectAgent/NullIntersectAgent.h>

bool NullIntersectAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	return false;
}

bool NullIntersectAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	return false;
}
