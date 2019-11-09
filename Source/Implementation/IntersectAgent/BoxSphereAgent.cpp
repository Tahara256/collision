#include <Collider/Implementation/IntersectAgent/BoxSphereAgent.h>
#include <Collider/Implementation/IntersectAgent/SphereBoxAgent.h>

bool BoxSphereAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	static const auto agent = SphereBoxAgent();
	return agent.intersect(collider2, collider1, outHitPos2, outHitPos1);
}

bool BoxSphereAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	static const auto agent = SphereBoxAgent();
	return agent.intersectTrigger(collider2, collider1);
}
