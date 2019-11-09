#include <Collider/Implementation/IntersectAgent/TerrainAnyColliderAgent.h>
#include <Collider/Implementation/IntersectAgent/AnyColliderTerrainAgent.h>

bool TerrainAnyColliderAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	static const auto agent = AnyColliderTerrainAgent();
	return agent.intersect(collider2, collider1, outHitPos2, outHitPos1);
}

bool TerrainAnyColliderAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	static const auto agent = AnyColliderTerrainAgent();
	return agent.intersectTrigger(collider2, collider1);
}
