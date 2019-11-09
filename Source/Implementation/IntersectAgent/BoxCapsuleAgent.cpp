#include <Collider/Implementation/IntersectAgent/BoxCapsuleAgent.h>
#include <Collider/Implementation/IntersectAgent/CapsuleBoxAgent.h>

bool BoxCapsuleAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	static const auto agent = CapsuleBoxAgent();
	return agent.intersect(collider2, collider1, outHitPos2, outHitPos1);
}

bool BoxCapsuleAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	static const auto agent = CapsuleBoxAgent();
	return agent.intersectTrigger(collider2, collider1);
}
