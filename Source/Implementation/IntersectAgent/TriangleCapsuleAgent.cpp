#include <Collider/Implementation/IntersectAgent/TriangleCapsuleAgent.h>
#include <Collider/Implementation/IntersectAgent/CapsuleTriangleAgent.h>

bool TriangleCapsuleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	static const auto agent = CapsuleTriangleAgent();
	return agent.intersect(collider2, collider1, outHitPos2, outHitPos1);
}

bool TriangleCapsuleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	static const auto agent = CapsuleTriangleAgent();
	return agent.intersectTrigger(collider2, collider1);
}
