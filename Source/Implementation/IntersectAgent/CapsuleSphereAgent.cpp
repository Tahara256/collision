#include <Collider/Implementation/IntersectAgent/CapsuleSphereAgent.h>
#include <Collider/Implementation/IntersectAgent/SphereCapsuleAgent.h>

bool CapsuleSphereAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	static const auto agent = SphereCapsuleAgent();
	return agent.intersect(collider2, collider1, outHitPos2, outHitPos1);
}

bool CapsuleSphereAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	static const auto agent = SphereCapsuleAgent();
	return agent.intersectTrigger(collider2, collider1);
}
