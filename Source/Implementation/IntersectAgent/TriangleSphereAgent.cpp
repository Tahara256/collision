#include <Collider/Implementation/IntersectAgent/TriangleSphereAgent.h>
#include <Collider/Implementation/IntersectAgent/SphereTriangleAgent.h>

bool TriangleSphereAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	static const auto agent = SphereTriangleAgent();
	return agent.intersect(collider2, collider1, outHitPos2, outHitPos1);
}

bool TriangleSphereAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	static const auto agent = SphereTriangleAgent();
	return agent.intersectTrigger(collider2, collider1);
}
