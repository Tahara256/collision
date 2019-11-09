#include <Collider/Implementation/IntersectAgent/AnyColliderTerrainAgent.h>
#include <Collider/Collider/Collider.h>
#include <Collider/CollisionShape/CapsuleShape.h>
#include <Collider/CollisionShape/TerrainShape.h>
#include <Collider/Implementation/Collision3D.h>
#include <Collider/Implementation/CollisionUtility.h>

using namespace std;
using namespace CollisionUtility;

bool AnyColliderTerrainAgent::intersect(const ColliderPtr & collider1, const ColliderPtr & collider2, Vector3 * outHitPos1, Vector3 * outHitPos2) const {
	// コライダーの形状取得
	decltype(auto) shape = collider2->getCollisionShape();
	const auto terrainShape = static_pointer_cast<TerrainShape>(shape);

	// 衝突判定
	const auto isCollided = collider1->intersectMesh(terrainShape->getTerrainMesh(), outHitPos1, outHitPos2);

	// より正しい衝突座標を取得
	if (isCollided)	TerrainCollisionDetail(collider1, terrainShape->getTerrainMesh(), outHitPos1, outHitPos2);

	return isCollided;
}

bool AnyColliderTerrainAgent::intersectTrigger(const ColliderPtr & collider1, const ColliderPtr & collider2) const {
	// コライダーの形状取得
	decltype(auto) shape = collider2->getCollisionShape();
	const auto terrainShape = static_pointer_cast<TerrainShape>(shape);

	// 衝突判定
	return collider1->intersectMesh(terrainShape->getTerrainMesh());
}
