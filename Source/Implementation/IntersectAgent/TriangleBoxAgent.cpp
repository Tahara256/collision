#include <Collider/Implementation/IntersectAgent/TriangleBoxAgent.h>
#include <Collider/Implementation/IntersectAgent/BoxTriangleAgent.h>

bool TriangleBoxAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	static const auto agent = BoxTriangleAgent();
	return agent.intersect(collider2, collider1, outHitPos2, outHitPos1);
}

bool TriangleBoxAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	static const auto agent = BoxTriangleAgent();
	return agent.intersectTrigger(collider2, collider1);
}
